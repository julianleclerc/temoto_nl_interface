#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import json
import time
import threading
import queue
from concurrent.futures import ThreadPoolExecutor
from nl_interface_msgs.srv import Chat

from temoto_nl_interface.ai import ai_core
from temoto_nl_interface.ai.prompts import chat_interface_prompt

MAX_CONTEXT = 10  # Sets number of past questions considered
MAX_CONCURRENT_REQUESTS = 8

class Target:
    def __init__(self, target_name):
        self.target_name = target_name
        self.history = []
        self.history_lock = threading.Lock()

    def add_to_history(self, role, message):
        with self.history_lock:
            timestamp = time.time()
            self.history.append(json.dumps({
                "role": role, 
                "content": f"time: {timestamp}, target: {self.target_name}, {role}: {message}", 
                "time": timestamp
            }))
            if len(self.history) > MAX_CONTEXT*2:
                self.history = self.history[-MAX_CONTEXT*2:]

    def get_history(self):
        with self.history_lock:
            return list(self.history)

    def get_target_name(self):
        return self.target_name
    
class TargetManager:
    def __init__(self):
        self.targets = {}
        self.target_lock = threading.Lock()

    def get_or_create_target(self, target_name):
        """Get an existing target or create it if it doesn't exist in a thread-safe manner."""
        with self.target_lock:
            if target_name not in self.targets:
                self.targets[target_name] = Target(target_name)
            return self.targets[target_name]
            
    def shutdown(self):
        """Clean up resources during shutdown"""
        with self.target_lock:
            for target in self.targets.values():
                target.history.clear()
                target.history_lock = None
                self.get_logger().info(f"Cleared history for target: {target.get_target_name()}")
        self.targets.clear()
        self.target_lock = None
        self.get_logger().info("TargetManager shutdown complete")
        
class ChatInterface(Node):
    def __init__(self):
        super().__init__('chat_interface_node')
        
        # Create callback groups for concurrent callback execution
        self.input_callback_group = ReentrantCallbackGroup()
        self.publisher_callback_group = ReentrantCallbackGroup()
        self.memory_response_callback_group = ReentrantCallbackGroup()
        
        # Initialize the target manager
        self.target_manager = TargetManager()
        
        # Create a request queue
        self.request_queue = queue.Queue()
        
        # Subscribe to enqueue_request with callback group
        self.sub = self.create_subscription(
            String, 
            'chat_interface_input', 
            self.enqueue_request, 
            10,
            callback_group=self.input_callback_group
        )

        # Create a publisher for sending chat responses with callback group
        self.pub = self.create_publisher(
            String, 
            'chat_interface_feedback', 
            10,
            callback_group=self.publisher_callback_group
        )

        # Create thread pool for processing requests with increased capacity
        self.thread_pool = ThreadPoolExecutor(max_workers=MAX_CONCURRENT_REQUESTS)
        
        # Flag to signal worker thread to stop
        self.shutdown_flag = threading.Event()

        # Start worker thread to process the queue
        self.processing_thread = threading.Thread(target=self.process_queue, daemon=True)
        self.processing_thread.start()

        # Create action publisher with callback group
        self.umrf_actions_pub = self.create_publisher(
            String, 
            'chat_action', 
            10,
            callback_group=self.publisher_callback_group
        )
        
        # Import prompt.
        try:
            self.INSTRUCTIONS = chat_interface_prompt.getprompt()
            self.get_logger().debug('Fetched Prompt')
        except Exception as e:
            self.get_logger().error(f'Error loading prompt: {e}')
            self.shutdown_flag.set()
            self.shutdown()
            return
        
        # Service client for get mem info
        self.get_logger().debug('Creating service client for memory')
        self.memory_available = True
        self.memory_response = None
        self.memory_response_ready = threading.Event()
        
        self.getMemory_client = self.create_client(
            Chat, 
            'get_memory', 
            callback_group=self.memory_response_callback_group
        )
        
        if not self.getMemory_client:
            self.memory_available = False
            self.get_logger().error('Failed to create getMemory client')

        self.get_logger().info(f'Chat_interface node is active with {MAX_CONCURRENT_REQUESTS} concurrent workers.')

    def enqueue_request(self, msg):
        """Add incoming messages to the processing queue."""
        try:
            # Check if message is valid before adding to queue
            json.loads(msg.data)
            self.get_logger().info(f"Received chat message, adding to queue")
            self.request_queue.put(msg)
            self.get_logger().debug(f"Queue size: {self.request_queue.qsize()}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid message format, not adding to queue: {e}")
            error_msg = String()
            error_msg.data = json.dumps({"targets": [], "type": "error", "message": f"Invalid message format: {e}"})
            self.pub.publish(error_msg)

    def process_queue(self):
        """Process messages from the queue using the thread pool."""
        while not self.shutdown_flag.is_set() and rclpy.ok():
            try:
                # Get message from queue (blocking with timeout)
                msg = self.request_queue.get(timeout=1.0)
                
                # Submit task to thread pool
                self.thread_pool.submit(self.handle_chat, msg)
                
                # Mark task as done
                self.request_queue.task_done()
            except queue.Empty:
                # No message in queue, continue waiting
                continue
            except Exception as e:
                self.get_logger().error(f"Error in queue processing: {e}")

        self.get_logger().info("Queue processing thread exiting")

    def handle_chat(self, msg):
        """Process chat request in a worker thread."""
        thread_id = threading.get_ident() 
        self.get_logger().info(f"Processing chat message in thread {thread_id}: {msg.data}")

        # Initialize variables that might be referenced in exception handlers
        targets_list = []
        target_objects = []
        error_response = None 

        try:
            request_message_str = msg.data
            request_message_json = json.loads(request_message_str)
            request_message = request_message_json.get("message", "")
            targets_list = request_message_json.get("targets", [])
            message_type = request_message_json.get("type", "request")

            # Check for request type
            if message_type != "request":
                self.get_logger().warn('Not a request message, ignoring.')
                return
            
            # Handle empty messages.
            if not request_message:
                self.get_logger().warn('Received empty message.')
                error_msg = String()
                error_msg.data = json.dumps({"targets": targets_list, "type": "error", "message": "No message received."})
                self.pub.publish(error_msg)
                return

            # Process targets and retrieve or create them
            target_objects = []
            for target_name in targets_list:
                target = self.target_manager.get_or_create_target(target_name)
                target_objects.append(target)

            # If no targets specified 
            if not target_objects:
                self.get_logger().warn('No targets specified.')
                error_msg = String()
                error_msg.data = json.dumps({"targets": [], "type": "error", "message": "No targets specified."})
                self.pub.publish(error_msg)
                return

            # Add the message to each target's history
            for target in target_objects:
                target.add_to_history("user", request_message)

            # Collect conversation history from all targets
            conversation_history = []
            for target in target_objects:
                target_history = target.get_history()
                if target_history:
                    self.get_logger().debug(f'Adding target history: {target_history}')
                    conversation_history.extend(target_history)
                else:
                    self.get_logger().warn(f'No history for target: {target.get_target_name()}')

            # Order history by time with error handling
            try:
                conversation_history.sort(key=lambda x: json.loads(x).get("time", 0))
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Failed to sort history by time: {e}")
                # Continue with unsorted history rather than failing
            
            self.get_logger().debug(f'Ordered conversation history: {conversation_history}')
             
            # Compile messages for the AI
            # Start with system instructions.
            messages = [
                {"role": "system", "content": self.INSTRUCTIONS},
            ]

            # Add previous conversation history.
            if conversation_history:
                recent_history = []
                # Use consistent number of messages 
                messages_to_include = min(len(conversation_history), MAX_CONTEXT*2)
                # Convert JSON strings back to dictionaries for the API
                for history_item in conversation_history[-messages_to_include:]:
                    try:
                        history_dict = json.loads(history_item)
                        recent_history.append(history_dict)
                    except json.JSONDecodeError:
                        self.get_logger().warn(f"Failed to parse history item: {history_item}")
                        
                self.get_logger().debug(f'Adding target conversational history: {recent_history}')
                messages.extend(recent_history)

            # Add the new user message.
            self.get_logger().debug(f'Appending request message: {request_message}')
            messages.append({"role": "user", "content": request_message})

            # Add memory to the prompt if available
            if self.memory_available and self.getMemory_client.service_is_ready():
                self.get_logger().debug('Fetching memory information')
                
                # Reset memory response state
                self.memory_response = None
                self.memory_response_ready.clear()
                
                # Prepare request
                memory_request = Chat.Request()
                memory_request.message = json.dumps({"targets": targets_list, "type": "memory"})
                
                # Send request asynchronously
                future = self.getMemory_client.call_async(memory_request)
                future.add_done_callback(self.memory_callback)
                
                # Wait for memory response with timeout
                if self.memory_response_ready.wait(timeout=5.0):
                    if self.memory_response:
                        # Use "system" role instead of "memory" since "memory" is not supported
                        messages.append({"role": "system", "content": f"Memory information: {self.memory_response}"})
                        self.get_logger().debug('Memory information added to messages as system message')
                    else:
                        # Skip adding memory info if it failed
                        self.get_logger().warn('Memory request failed, not adding to messages')
                else:
                    self.get_logger().warn('Memory request timed out, not adding to messages')
            else:
                self.get_logger().debug('Memory not available, skipping memory fetch.')
            
            # Generate chat response with the collected messages
            self.generate_chat_response(messages, targets_list, target_objects)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing request JSON: {e}')
            error_response = String()
            error_response.data = json.dumps({
                "targets": targets_list, 
                "type": "error", 
                "message": f"Invalid request format: {e}"
            })
            self.pub.publish(error_response)
        except Exception as e:
            self.get_logger().error(f'Unexpected error in chat processing: {e}')
            error_response = String()
            error_response.data = json.dumps({
                "targets": targets_list, 
                "type": "error", 
                "message": f"Unexpected error: {e}"
            })
            self.pub.publish(error_response)

    def memory_callback(self, future):
        """Callback for memory service responses"""
        try:
            response = future.result()
            # Check what attribute is actually available in the Chat.Response object
            # The error suggests 'message' isn't the correct attribute name
            if hasattr(response, 'message'):
                self.memory_response = response.message
            elif hasattr(response, 'response'):
                self.memory_response = response.response
            elif hasattr(response, 'data'):
                self.memory_response = response.data
            else:
                # Log all available attributes for debugging
                self.get_logger().error(f"Unknown response structure. Available attributes: {dir(response)}")
                self.memory_response = None
        except Exception as e:
            self.get_logger().error(f"Error in memory callback: {e}")
            self.memory_response = None
        finally:
            # Signal that we have a response (or failed to get one)
            self.memory_response_ready.set()

    def generate_chat_response(self, messages, targets_list, target_objects):
        """Generate and process AI response to chat messages"""
        # Initialize variables for chat response and JSON response
        chat_response = None
        json_response = None
        tic = time.time()

        try:
            assistant_reply = ai_core.AI_Image_Prompt(
                messages,
                TEMPERATURE=0.1,
                MAX_TOKENS=2048,
                FREQUENCY_PENALTY=0.0,
                PRESENCE_PENALTY=0.0
            )
            tac = time.time()
            self.get_logger().info(f'Received response in {tac - tic:.2f} seconds')
            self.get_logger().info(f'Assistant reply: {assistant_reply}')
        except Exception as e:
            self.get_logger().error(f"Error calling AI model: {e}")
            error_response = String()
            error_response.data = json.dumps({
                "targets": targets_list, 
                "type": "error", 
                "message": f"Failed to get AI response: {e}"
            })
            self.pub.publish(error_response)
            return

        try:
            # Attempt to parse the assistant reply.
            if isinstance(assistant_reply, str):
                assistant_reply = self.llm_response_parser(assistant_reply)
                json_response = json.loads(assistant_reply)
                chat_response = json_response.get("message_response")
                if not chat_response:
                    raise KeyError("message_response not found in AI response")
                self.get_logger().debug('Assistant reply parsed as JSON')
            else:
                raise ValueError(f"Unexpected response type: {type(assistant_reply)}")

            # Update the history for each target
            for target in target_objects:
                target.add_to_history("assistant", chat_response)

            # Publish the chat response to user interface.
            response_msg = String()
            response_msg.data = json.dumps({
                "targets": targets_list, 
                "type": "response", 
                "message": chat_response
            })
            self.pub.publish(response_msg)
            self.get_logger().info(f'Published chat response: {response_msg.data}')

            # Publish the generated umrf actions to umrf planner.
            if json_response:
                json_response["targets"] = targets_list
                umrf_msg = String()
                umrf_msg.data = json.dumps(json_response)
                self.umrf_actions_pub.publish(umrf_msg)
                self.get_logger().info(f'Published UMRF actions: {umrf_msg.data}')
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON parsing error: {e}')
            # Still update the history with the error
            for target in target_objects:
                target.add_to_history("assistant", f"Error: JSON parsing failed")
                
            error_response = String()
            error_response.data = json.dumps({
                "targets": targets_list, 
                "type": "error", 
                "message": f"Invalid JSON response format: {e}"
            })
            self.pub.publish(error_response)
        except KeyError as e:
            self.get_logger().error(f'Missing key in response: {e}')
            for target in target_objects:
                target.add_to_history("assistant", f"Error: Missing key in response")
                
            error_response = String()
            error_response.data = json.dumps({
                "targets": targets_list, 
                "type": "error", 
                "message": f"Response missing required key: {e}"
            })
            self.pub.publish(error_response)
        except ValueError as e:
            self.get_logger().error(f'Invalid value in response: {e}')
            for target in target_objects:
                target.add_to_history("assistant", f"Error: Invalid value in response")
                
            error_response = String()
            error_response.data = json.dumps({
                "targets": targets_list, 
                "type": "error", 
                "message": f"Invalid response value: {e}"
            })
            self.pub.publish(error_response)

    def llm_response_parser(self, message: str) -> str:
        """
        Simple function to extract JSON content from LLM responses.
        It takes everything between the first '{' and the last '}' and tries to parse it.
        Returns a fallback JSON with 'success': 'false' if parsing fails.
        """
        self.get_logger().debug(f"Parsing JSON from message of length {len(message)}")
        
        try:
            # First, try parsing the entire message as JSON (handles clean responses)
            try:
                parsed = json.loads(message)
                self.get_logger().debug("Parsed entire message as valid JSON")
                return json.dumps(parsed)
            except json.JSONDecodeError:
                self.get_logger().debug("Message is not valid JSON, trying extraction")
            
            # Extract content between first { and last }
            start = message.find('{')
            end = message.rfind('}')
            
            if start != -1 and end != -1 and end > start:
                # Extract the content
                json_str = message[start:end + 1]
                self.get_logger().debug(f"Extracted content from index {start} to {end}")
                
                # Try to parse the extracted content
                try:
                    parsed = json.loads(json_str)
                    self.get_logger().debug("Successfully parsed extracted content")
                    return json.dumps(parsed)
                except json.JSONDecodeError as je:
                    self.get_logger().error(f"Failed to parse extracted content: {je}")
                    self.get_logger().debug(f"Problematic content: {json_str[:100]}...")
            else:
                self.get_logger().error("No JSON-like content found in message")
        
        except Exception as e:
            self.get_logger().error(f"Unexpected error in JSON parser: {e}")
        
        # Return fallback JSON if all parsing attempts fail
        fallback = {
            "success": "false",
            "target": getattr(self, "target", "unknown"),
            "message": "Failed to parse LLM response",
            "queue": [],
            "system_cmd": []
        }
        self.get_logger().warning("Returning fallback JSON due to parsing failure")
        return json.dumps(fallback)
        
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.get_logger().info("Shutting down chat interface node")
        if hasattr(self, 'target_manager'):
            self.target_manager.shutdown()
        super().destroy_node()

    def shutdown(self):
        """Clean shutdown of threads and resources"""
        self.get_logger().info("Shutting down ChatInterface node")
        self.shutdown_flag.set()
        self.processing_thread.join(timeout=5.0)
        self.thread_pool.shutdown(wait=True)
        self.get_logger().info("All threads and resources cleaned up")

def main(args=None):
    rclpy.init(args=args)
    node = ChatInterface()
    
    # Create a MultiThreadedExecutor with more threads to handle concurrent callbacks
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        # Use the executor instead of rclpy.spin
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('ChatInterface node has been shut down.')
    finally:
        # Clean shutdown
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()