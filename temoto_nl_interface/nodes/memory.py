#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nl_interface_msgs.srv import Chat
import threading
import json
import time

from temoto_nl_interface.ai import ai_core
from temoto_nl_interface.ai.prompts import memory_prompt



class Memory:
    """Class to encapsulate a single memory entry with type, data, and info"""
    
    def __init__(self, name, memory_type, data, info):
        self.name = name
        self.type = memory_type
        self.data = data
        self.info = info
        self.created_at = time.time()
        self.last_accessed = time.time()
        self.access_count = 0
    
    def access(self):
        """Update access statistics when memory is accessed"""
        self.last_accessed = time.time()
        self.access_count += 1
    
    def update(self, memory_type=None, data=None, info=None):
        """Update memory fields if provided"""
        if memory_type is not None:
            self.type = memory_type
        if data is not None:
            self.data = data
        if info is not None:
            self.info = info
        self.last_accessed = time.time()
    
    def to_dict(self):
        """Convert memory to dictionary for storage/transmission"""
        return {
            "type": self.type,
            "data": self.data,
            "info": self.info
        }
    
    def get_info_dict(self):
        """Get just the name and info for LLM queries"""
        return {
            "name": self.name,
            "info": self.info
        }


class MemoryManager:
    """Manager class to handle multiple memories with thread safety"""
    
    def __init__(self, logger):
        self.memories = {}
        self.lock = threading.RLock()
        self.logger = logger
    
    def create_memory(self, name, memory_type, data, info):
        """Create a new memory entry"""
        with self.lock:
            memory = Memory(name, memory_type, data, info)
            self.memories[name] = memory
            self.logger.info(f"Created new memory: {name}")
            return memory
    
    def get_memory(self, name):
        """Get a memory by name"""
        with self.lock:
            memory = self.memories.get(name)
            if memory:
                memory.access()
            return memory
    
    def update_memory(self, name, memory_type=None, data=None, info=None):
        """Update an existing memory"""
        with self.lock:
            memory = self.memories.get(name)
            if memory:
                memory.update(memory_type, data, info)
                self.logger.info(f"Updated memory: {name}")
                return True
            return False
    
    def delete_memory(self, name):
        """Delete a memory"""
        with self.lock:
            if name in self.memories:
                del self.memories[name]
                self.logger.info(f"Deleted memory: {name}")
                return True
            return False
    
    def get_all_memories(self):
        """Get all memories as a dictionary"""
        with self.lock:
            return {name: memory.to_dict() for name, memory in self.memories.items()}
    
    def get_memory_info_for_llm(self):
        """Get memory info dictionary for LLM queries"""
        with self.lock:
            return {name: memory.get_info_dict() for name, memory in self.memories.items()}


class MemoryNode(Node):
    def __init__(self):
        super().__init__('memory_node')

        # Create callback groups
        self.service_callback_group = ReentrantCallbackGroup()
        self.subscription_callback_group = ReentrantCallbackGroup()

        # Initialize memory manager
        self.memory_manager = MemoryManager(self.get_logger())

        # get_memory_info service to return the global memory info
        self.srv = self.create_service(
            Chat,
            'get_memory',
            self.handle_memory_request,
            callback_group=self.service_callback_group
        )

        # Subscription to add to memory
        self.add_to_memory = self.create_subscription(
            String,
            '/add_to_memory',
            self.add_to_memory_callback,
            10,
            callback_group=self.subscription_callback_group
        )
        
        
        ## Add to memory content in data


        self.get_logger().info('Memory Node is active')
    
    def add_to_memory_callback(self, msg):
        """
        Callback function to add the incoming message to the memory
        """
        self.get_logger().info(f'Received memory update: {msg.data}')
        
        try:
            # Parse the incoming message
            message_data = json.loads(msg.data)
            
            # Iterate through the message data
            for key, value in message_data.items():
                memory_type = value.get("type", "unknown")
                data = value.get("data", "")
                info = value.get("info", "")
                
                # Check if memory exists and update, or create new
                if self.memory_manager.get_memory(key):
                    self.memory_manager.update_memory(key, memory_type, data, info)
                else:
                    self.memory_manager.create_memory(key, memory_type, data, info)
                    
            self.get_logger().info(f'Memory updated successfully')
            
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse message: Invalid JSON format')
        except Exception as e:
            self.get_logger().error(f'Error processing memory update: {str(e)}')

    def handle_memory_request(self, request, response):
        """
        Callback function to handle the get_memory_info request
        Returns memory data based on LLM analysis of the request
        """
        self.get_logger().info(f'Received memory request: {request.message}')

        try:
            data = json.loads(request.message)
            request_message = data.get("request", "return all memories")      
            
            # If no memories are available, return an empty response
            if not self.memory_manager.get_all_memories():
                self.get_logger().info('No memories available, returning empty response')
                response.response = json.dumps({})
                return response

            # If the request is "return all memories", return all memories
            if request_message == "return all memories":
                self.get_logger().info('Returning all memories')
                response.response = json.dumps(self.memory_manager.get_all_memories())
                return response
            
            # Get memory info for LLM analysis
            memory_info = self.memory_manager.get_memory_info_for_llm()
            
            # Prepare the messages for AI
            instructions = memory_prompt.getprompt()
            messages = [
                {"role": "system", "content": instructions},
                {"role": "assistant", "content": request_message},
                {"role": "user", "content": json.dumps(memory_info)}
            ]
            self.get_logger().info(f"Prepared messages for AI analysis")

            # AI parameters
            temperature = 0.2
            max_tokens = 1024
            frequency_penalty = 0
            presence_penalty = 0.1

            # Call AI to handle the request
            self.get_logger().info("Calling AI to analyze memory request...")
            assistant_reply = ai_core.AI_Image_Prompt(
                messages, temperature, max_tokens, frequency_penalty, presence_penalty)
            self.get_logger().info(f"Received AI response for memory selection")

            # Prepare the response
            try:
                # Parse the requested memory names from the AI response
                requested_memories = json.loads(assistant_reply)

                # Create a new dictionary with only the requested memories
                memory_data = {}
                for memory_name in requested_memories:
                    memory = self.memory_manager.get_memory(memory_name)
                    if memory:
                        memory_data[memory_name] = memory.to_dict()

                # Convert the memory_data dictionary to a JSON string       
                response.response = json.dumps(memory_data)
                self.get_logger().info(f'Responding with requested memory data for: {requested_memories}')

            except json.JSONDecodeError:
                self.get_logger().error('Failed to parse AI response: Invalid JSON format')
                response.response = json.dumps({"error": "Invalid AI response format"})
            except Exception as e:
                self.get_logger().error(f'Error processing AI response: {str(e)}')
                response.response = json.dumps({"error": f"Error processing request: {str(e)}"})
                
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse request: Invalid JSON format')
            response.response = json.dumps({"error": "Invalid JSON format in request"})
        except Exception as e:
            self.get_logger().error(f'Error processing request: {str(e)}')
            response.response = json.dumps({"error": f"Error processing request: {str(e)}"})
            
        return response


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = MemoryNode()

    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped via keyboard interrupt.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()