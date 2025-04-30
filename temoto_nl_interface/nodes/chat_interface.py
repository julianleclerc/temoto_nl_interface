#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import json
import time
import threading
from nl_interface_msgs.srv import Chat

from temoto_nl_interface.ai import ai_core
from temoto_nl_interface.ai.prompts import chat_interface_prompt

# Configuration constants
MAX_CONTEXT = 10  # Sets number of past questions considered
MEMORY_TIMEOUT = 3.0 

class Target:
    """Represents a single conversation target with thread-safe history management"""
    def __init__(self, target_name):
        self.target_name = target_name
        self.history = []
        self.history_lock = threading.Lock()
        self.last_access_time = time.time()

    def add_to_history(self, role, message):
        """Add a message to the target's history in a thread-safe manner"""
        with self.history_lock:
            timestamp = time.time()
            self.last_access_time = timestamp
            self.history.append(json.dumps({
                "role": role, 
                "content": f"time: {timestamp}, target: {self.target_name}, {role}: {message}", 
                "time": timestamp
            }))
            # Limit history size
            if len(self.history) > MAX_CONTEXT*2:
                self.history = self.history[-MAX_CONTEXT*2:]

    def get_history(self):
        """Get a copy of the target's history in a thread-safe manner"""
        with self.history_lock:
            return list(self.history)

    def get_target_name(self):
        """Get the target's name"""
        return self.target_name
    
    def get_last_access_time(self):
        """Get the time of last access to this target"""
        with self.history_lock:
            return self.last_access_time
    
class TargetManager:
    """Manages multiple conversation targets with thread-safety"""
    def __init__(self, logger):
        self.targets = {}
        self.target_lock = threading.Lock()
        self.logger = logger

    def get_or_create_target(self, target_name):
        """Get an existing target or create it if it doesn't exist in a thread-safe manner"""
        with self.target_lock:
            if target_name not in self.targets:
                self.targets[target_name] = Target(target_name)
                self.logger.info(f"Created new target: {target_name}")
            return self.targets[target_name]
    
    def get_all_targets(self):
        """Get a list of all target names"""
        with self.target_lock:
            return list(self.targets.keys())
            
    def shutdown(self):
        """Clean up resources during shutdown"""
        with self.target_lock:
            target_count = len(self.targets)
            for target in self.targets.values():
                target.history.clear()
            self.targets.clear()
            self.logger.info(f"Cleared {target_count} targets during shutdown")
        
class ChatInterface(Node):
    def __init__(self):
        super().__init__('chat_interface_node')
        
        # Logger configuration - control log levels
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # Create callback groups for concurrent callback execution
        self.input_callback_group = ReentrantCallbackGroup()
        self.publisher_callback_group = ReentrantCallbackGroup()
        self.memory_callback_group = ReentrantCallbackGroup()
        
        # Initialize the target manager with logger
        self.target_manager = TargetManager(self.get_logger())
        
        # Subscribe to chat input with callback group - direct processing
        self.sub = self.create_subscription(
            String, 
            'chat_interface_input', 
            self.handle_chat, 
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

        # Create action publisher with callback group
        self.umrf_actions_pub = self.create_publisher(
            String, 
            'chat_action', 
            10,
            callback_group=self.publisher_callback_group
        )
        
        # Import prompt
        try:
            self.INSTRUCTIONS = chat_interface_prompt.getprompt()
            self.get_logger().debug('Fetched prompt template successfully')
        except Exception as e:
            self.get_logger().error(f'Error loading prompt: {e}')
            return
        
        # Service client for memory
        self.get_logger().debug('Creating service client for memory')
        self.memory_available = True
        
        self.getMemory_client = self.create_client(
            Chat, 
            'get_memory', 
            callback_group=self.memory_callback_group
        )
        
        if not self.getMemory_client:
            self.memory_available = False
            self.get_logger().error('Failed to create getMemory client')

        self.get_logger().info('Chat interface node is active with direct processing')

    def handle_chat(self, msg):
        """Process incoming chat messages directly in callback thread"""
        thread_id = threading.get_ident() 
        start_time = time.time()
        self.get_logger().info(f"Processing chat message in thread {thread_id}")

        # Initialize variables that might be referenced in exception handlers
        targets_list = []
        target_objects = []
        error_response = None 

        try:
            # Extract request data
            request_data = self.parse_request_message(msg.data)
            if not request_data:
                return
                
            request_message = request_data.get("message", "")
            targets_list = request_data.get("targets", [])
            message_type = request_data.get("type", "request")

            # Validate the request
            if not self.validate_request(message_type, request_message, targets_list):
                return
            
            # Process targets and create conversation context
            target_objects = self.prepare_target_objects(targets_list)
            if not target_objects:
                return
                
            # Update target histories
            self.update_target_histories(target_objects, "user", request_message)
            
            # Build conversation context
            messages = self.build_conversation_context(target_objects, request_message)
            
            # Fetch memory if available (non-blocking)
            memory_content = self.fetch_memory_async(targets_list)
            if memory_content:
                messages.append({"role": "system", "content": f"Memory information: {memory_content}"})
            
            # Generate and process the response
            self.generate_and_process_response(messages, targets_list, target_objects)
            
            # Log completion time
            processing_time = time.time() - start_time
            self.get_logger().info(f"Request processed in {processing_time:.2f} seconds")

        except Exception as e:
            self.get_logger().error(f'Unexpected error in chat processing: {e}')
            
            # Attempt to send error response
            self.send_error_response(targets_list, f"Unexpected error: {e}")

    def parse_request_message(self, message_str):
        """Parse the JSON request message"""
        try:
            return json.loads(message_str)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing request JSON: {e}')
            self.send_error_response([], f"Invalid request format: {e}")
            return None

    def validate_request(self, message_type, request_message, targets_list):
        """Validate the request data"""
        # Check for request type
        if message_type != "request":
            self.get_logger().warn('Not a request message, ignoring.')
            return False
        
        # Handle empty messages
        if not request_message:
            self.get_logger().warn('Received empty message.')
            self.send_error_response(targets_list, "No message received.")
            return False
            
        return True

    def prepare_target_objects(self, targets_list):
        """Process targets and retrieve or create them"""
        target_objects = []
        for target_name in targets_list:
            target = self.target_manager.get_or_create_target(target_name)
            target_objects.append(target)

        # If no targets specified
        if not target_objects:
            self.get_logger().warn('No targets specified.')
            self.send_error_response([], "No targets specified.")
            return None
            
        return target_objects

    def update_target_histories(self, target_objects, role, message):
        """Add the message to each target's history"""
        for target in target_objects:
            target.add_to_history(role, message)

    def build_conversation_context(self, target_objects, request_message):
        """Build the conversation context from target histories"""
        # Collect conversation history from all targets
        conversation_history = []
        for target in target_objects:
            target_history = target.get_history()
            if target_history:
                self.get_logger().debug(f'Adding history for target: {target.get_target_name()}')
                conversation_history.extend(target_history)
            else:
                self.get_logger().debug(f'No history for target: {target.get_target_name()}')

        # Order history by time with error handling
        try:
            conversation_history.sort(key=lambda x: json.loads(x).get("time", 0))
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to sort history by time: {e}")
            # Continue with unsorted history rather than failing
        
        # Compile messages for the AI
        messages = [
            {"role": "system", "content": self.INSTRUCTIONS},
        ]

        # Add previous conversation history
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
                    
            self.get_logger().debug(f'Adding conversational history, {len(recent_history)} items')
            messages.extend(recent_history)

        # Add the new user message
        messages.append({"role": "user", "content": request_message})
        
        return messages

    def fetch_memory_async(self, targets_list):
        """Fetch memory information asynchronously"""
        if not self.memory_available or not self.getMemory_client.service_is_ready():
            self.get_logger().debug('Memory service not available, skipping memory fetch')
            return None
            
        self.get_logger().debug('Fetching memory information asynchronously')
        
        # Create a synchronization event
        memory_response_ready = threading.Event()
        memory_content = [None]  # List to store response (to avoid nonlocal in Python 2)
        
        # Callback for memory response
        def memory_callback(future):
            try:
                response = future.result()
                if hasattr(response, 'response'):
                    memory_content[0] = response.response
                else:
                    self.get_logger().error(f"Unknown memory response structure. Available attributes: {dir(response)}")
            except Exception as e:
                self.get_logger().error(f"Error in memory callback: {e}")
            finally:
                memory_response_ready.set()
        
        # Prepare and send request
        memory_request = Chat.Request()
        memory_request.message = json.dumps({"targets": targets_list, "type": "memory"})
        future = self.getMemory_client.call_async(memory_request)
        future.add_done_callback(memory_callback)
        
        # Wait for response with timeout
        if memory_response_ready.wait(timeout=MEMORY_TIMEOUT):
            if memory_content[0]:
                self.get_logger().debug('Memory information received')
                return memory_content[0]
            else:
                self.get_logger().warn('Memory request failed to produce content')
        else:
            self.get_logger().warn('Memory request timed out')
            
        return None

    def generate_and_process_response(self, messages, targets_list, target_objects):
        """Generate chat response and process it"""
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
            self.get_logger().info(f'Received AI response in {tac - tic:.2f} seconds')
        except Exception as e:
            self.get_logger().error(f"Error calling AI model: {e}")
            self.send_error_response(targets_list, f"Failed to get AI response: {e}")
            return

        try:
            # Parse the assistant reply
            assistant_reply = self.llm_response_parser(assistant_reply)
            json_response = json.loads(assistant_reply)
            chat_response = json_response.get("message_response")
            
            if not chat_response:
                raise KeyError("message_response not found in AI response")
                
            # Update target histories
            self.update_target_histories(target_objects, "assistant", chat_response)

            # Publish the chat response to user interface
            self.send_chat_response(targets_list, chat_response)

            # Publish the generated UMRF actions
            if json_response:
                json_response["targets"] = targets_list
                self.publish_umrf_actions(json_response)
        
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            error_msg = f"Error processing AI response: {e}"
            self.get_logger().error(error_msg)
            
            # Update target histories with error
            for target in target_objects:
                target.add_to_history("assistant", f"Error: {error_msg}")
                
            # Send error response
            self.send_error_response(targets_list, error_msg)

    def send_chat_response(self, targets_list, message):
        """Send a chat response message"""
        response_msg = String()
        response_msg.data = json.dumps({
            "targets": targets_list, 
            "type": "response", 
            "message": message
        })
        self.pub.publish(response_msg)
        self.get_logger().info('Published chat response')

    def send_error_response(self, targets_list, error_message):
        """Send an error response message"""
        error_response = String()
        error_response.data = json.dumps({
            "targets": targets_list, 
            "type": "error", 
            "message": error_message
        })
        self.pub.publish(error_response)
        self.get_logger().info(f'Published error response: {error_message}')

    def publish_umrf_actions(self, json_response):
        """Publish UMRF actions to the action topic"""
        umrf_msg = String()
        umrf_msg.data = json.dumps(json_response)
        self.umrf_actions_pub.publish(umrf_msg)
        self.get_logger().info('Published UMRF actions')

    def llm_response_parser(self, message: str) -> str:
        """
        Extract JSON content from LLM responses.
        It takes everything between the first '{' and the last '}' and tries to parse it.
        Returns a fallback JSON with 'success': 'false' if parsing fails.
        """
        self.get_logger().debug(f"Parsing JSON from message of length {len(message)}")
        
        try:
            # First, try parsing the entire message as JSON
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
                self.get_logger().debug(f"Extracted JSON content")
                
                # Try to parse the extracted content
                try:
                    parsed = json.loads(json_str)
                    self.get_logger().debug("Successfully parsed extracted content")
                    return json.dumps(parsed)
                except json.JSONDecodeError as je:
                    self.get_logger().error(f"Failed to parse extracted content: {je}")
            else:
                self.get_logger().error("No JSON-like content found in message")
        
        except Exception as e:
            self.get_logger().error(f"Unexpected error in JSON parser: {e}")
        
        # Return fallback JSON if all parsing attempts fail
        fallback = {
            "success": "false",
            "message_response": "Sorry, I encountered an error processing your request.",
            "queue": []
        }
        self.get_logger().warning("Returning fallback JSON due to parsing failure")
        return json.dumps(fallback)
        
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.get_logger().info("Shutting down chat interface node")
        if hasattr(self, 'target_manager'):
            self.target_manager.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ChatInterface()
    
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('ChatInterface node has been shut down.')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()