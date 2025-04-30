#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from nl_interface_msgs.srv import Chat
from temoto_nl_interface.ai import ai_core
from temoto_nl_interface.ai.prompts import error_handling_prompt

import re
import json
import threading
import time
import queue
from enum import Enum, auto
from typing import Dict, List, Any, Optional, Callable

class TargetState(Enum):
    CREATED = auto()
    WAITING_FOR_MEMORY = auto()
    PROCESSING = auto()
    WAITING_FOR_USER = auto()
    RESOLVED = auto()
    FAILED = auto()
    SKIPPED = auto()

class TargetErrorState:
    """Class to encapsulate and manage the state of a single target error handling process"""
    
    def __init__(self, target_id: str, error_message: str, action_called: str, 
                 runtime_messages: str, umrf_queue: str, 
                 process_data_text: str, process_data_images: str,
                 process_auto_solve: str, process_use_mem: str,
                 auto_temperature: float, auto_max_tokens: int,
                 auto_frequency_penalty: float, auto_presence_penalty: float,
                 user_temperature: float, user_max_tokens: int,
                 user_frequency_penalty: float, user_presence_penalty: float):
        
        # Target identifiers and error info
        self.target_id = target_id
        self.error_message = error_message
        self.action_called = action_called
        self.runtime_messages = runtime_messages
        self.umrf_queue = umrf_queue
        self.process_data_text = process_data_text
        self.process_data_images = process_data_images
        self.process_auto_solve = process_auto_solve
        self.process_use_mem = process_use_mem
        
        # AI parameters
        self.auto_temperature = auto_temperature
        self.auto_max_tokens = auto_max_tokens
        self.auto_frequency_penalty = auto_frequency_penalty
        self.auto_presence_penalty = auto_presence_penalty
        self.user_temperature = user_temperature
        self.user_max_tokens = user_max_tokens
        self.user_frequency_penalty = user_frequency_penalty
        self.user_presence_penalty = user_presence_penalty
        
        # State management
        self.state = TargetState.CREATED
        self.state_lock = threading.RLock()
        self.messages = []  
        self.memory = "none"
        self.trial_nb = 0
        self.max_trials = 3
        self.last_interaction_time = time.time()
        self.creation_time = time.time()
        self.waiting_for_user_response = False
        
        # Processing flags
        self.is_being_processed = False

    def set_state(self, new_state: TargetState) -> None:
        """Thread-safe state update"""
        with self.state_lock:
            self.state = new_state
            self.last_interaction_time = time.time()
    
    def get_state(self) -> TargetState:
        """Thread-safe state access"""
        with self.state_lock:
            return self.state
    
    def add_message(self, role: str, content: str) -> None:
        """Thread-safe message addition"""
        with self.state_lock:
            self.messages.append({"role": role, "content": content})
            self.last_interaction_time = time.time()
    
    def set_memory(self, memory_content: str) -> None:
        """Thread-safe memory update"""
        with self.state_lock:
            self.memory = memory_content
            self.last_interaction_time = time.time()
    
    def increment_trial(self) -> int:
        """Thread-safe trial increment"""
        with self.state_lock:
            self.trial_nb += 1
            self.last_interaction_time = time.time()
            return self.trial_nb
    
    def set_waiting_for_user_response(self, waiting: bool) -> None:
        """Thread-safe update of waiting status"""
        with self.state_lock:
            self.waiting_for_user_response = waiting
            self.last_interaction_time = time.time()
    
    def get_waiting_for_user_response(self) -> bool:
        """Thread-safe access to waiting status"""
        with self.state_lock:
            return self.waiting_for_user_response
    
    def get_messages_copy(self) -> List[Dict[str, str]]:
        """Thread-safe copy of messages"""
        with self.state_lock:
            return self.messages.copy()
    
    def set_processing(self, is_processing: bool) -> None:
        """Thread-safe update of processing flag"""
        with self.state_lock:
            self.is_being_processed = is_processing
    
    def is_processing(self) -> bool:
        """Thread-safe access to processing flag"""
        with self.state_lock:
            return self.is_being_processed


class TargetErrorManager:
    """Manager class to handle multiple target error states efficiently with thread safety"""
    
    def __init__(self, logger):
        """Initialize the target error manager"""
        self.logger = logger
        self.targets: Dict[str, TargetErrorState] = {}
        self.targets_lock = threading.RLock()
        
    def add_target(self, target_state: TargetErrorState) -> None:
        """Add a new target to the manager"""
        with self.targets_lock:
            self.targets[target_state.target_id] = target_state
            self.logger.info(f"Added target {target_state.target_id} to manager")
    
    def get_target(self, target_id: str) -> Optional[TargetErrorState]:
        """Get a target state by ID"""
        with self.targets_lock:
            return self.targets.get(target_id)
    
    def remove_target(self, target_id: str) -> None:
        """Remove a target from the manager"""
        with self.targets_lock:
            if target_id in self.targets:
                del self.targets[target_id]
                self.logger.info(f"Removed target {target_id} from manager")
    
    def has_target(self, target_id: str) -> bool:
        """Check if a target exists in the manager"""
        with self.targets_lock:
            return target_id in self.targets
    
    def get_all_target_ids(self) -> List[str]:
        """Get a list of all target IDs"""
        with self.targets_lock:
            return list(self.targets.keys())


class ErrorHandler(Node):
    def __init__(self):
        super().__init__('error_handler_node')

        # Create target manager for thread-safe access
        self.target_manager = TargetErrorManager(self.get_logger())

        # Get the instructions
        self.INSTRUCTIONS = ""
        try:
            self.INSTRUCTIONS = error_handling_prompt.getprompt()
            self.get_logger().debug("Successfully retrieved instructions template")
        except Exception as e:
            self.get_logger().error(f"Unexpected error retrieving instructions: {e}")

        # Callback groups
        self.get_logger().debug('Creating callback groups')
        self.memory_cb_group = ReentrantCallbackGroup()  # For memory service
        self.processing_cb_group = ReentrantCallbackGroup()  # For error processing tasks
        self.subscriber_cb_group = ReentrantCallbackGroup()  # For subscriptions
        
        # Service client for get mem info
        self.get_logger().debug('Creating service client for memory')
        self.memory_available = True
        self.getMemory_client = self.create_client(
            Chat, 'get_memory', callback_group=self.memory_cb_group)
        if not self.getMemory_client:
            self.memory_available = False
            self.get_logger().error('Failed to create getMemory client')

        # UMRF Planning IO
        self.get_logger().debug('Setting up UMRF Planning IO')
        self.error_handling_message = self.create_subscription(
            String, '/error_handling_message', self.listener_callback, 10,
            callback_group=self.subscriber_cb_group)
        self.error_queue_update_pub = self.create_publisher(String, '/umrf_correction', 10)

        # Chat Interface IO
        self.get_logger().debug('Setting up Chat Interface IO')
        self.prompt_response_sub = self.create_subscription(
            String, '/chat_interface_input', self.prompt_response_callback, 10, 
            callback_group=self.subscriber_cb_group)
        self.prompt_request_pub = self.create_publisher(String, '/chat_interface_feedback', 10)
        self.action_status_pub = self.create_publisher(String, '/action_status', 10)

        # Timer for background processing
        self.processing_timer = self.create_timer(
            0.5,  # Check every 500ms
            self.process_targets,
            callback_group=self.processing_cb_group
        )
        
        self.get_logger().info('Error Handler node is active with ROS2 callback groups')

    def listener_callback(self, msg):
        """Handle incoming error messages"""
        self.get_logger().info(f"Received error handling message: {msg.data}")

        try:
            self.get_logger().debug("Parsing received JSON data")
            data = json.loads(msg.data)

            # Extract target from the message
            target = data.get("target", "none")
            if target == "none":
                self.get_logger().error("No target specified for error handling.")
                return
            
            # Extract all parameters
            error_message = data.get("error_message", "none")
            action_called = data.get("action_called", "none")
            runtime_messages = data.get("runtime_messages", "none")
            umrf_queue = data.get("umrf_queue", "none")
            process_data_text = data.get("process_data_text", "none")
            process_data_images = data.get("process_data_images", "none")
            process_auto_solve = data.get("process_auto_solve", "true")
            process_use_mem = data.get("process_use_mem", "true")
            
            auto_temperature = data.get("auto_temperature", 0.3)
            auto_max_tokens = data.get("auto_max_tokens", 2000)
            auto_frequency_penalty = data.get("auto_frequency_penalty", 0.0)
            auto_presence_penalty = data.get("auto_presence_penalty", 0.0)
            user_temperature = data.get("user_temperature", 0.3)
            user_max_tokens = data.get("user_max_tokens", 2000)
            user_frequency_penalty = data.get("user_frequency_penalty", 0.0)
            user_presence_penalty = data.get("user_presence_penalty", 0.0)

            # Create target error state
            target_state = TargetErrorState(
                target_id=target,
                error_message=error_message,
                action_called=action_called,
                runtime_messages=runtime_messages,
                umrf_queue=umrf_queue,
                process_data_text=process_data_text,
                process_data_images=process_data_images,
                process_auto_solve=process_auto_solve,
                process_use_mem=process_use_mem,
                auto_temperature=auto_temperature,
                auto_max_tokens=auto_max_tokens,
                auto_frequency_penalty=auto_frequency_penalty,
                auto_presence_penalty=auto_presence_penalty,
                user_temperature=user_temperature,
                user_max_tokens=user_max_tokens,
                user_frequency_penalty=user_frequency_penalty,
                user_presence_penalty=user_presence_penalty
            )
            
            # Add to manager - thread safe operation
            self.target_manager.add_target(target_state)
            self.get_logger().info(f"Created state for target: {target}")
            
            # Process memory if asked and available
            if process_use_mem == "true" and self.memory_available == True:
                target_state.set_state(TargetState.WAITING_FOR_MEMORY)
                self.request_memory_for_target(target)
            else:
                target_state.set_state(TargetState.PROCESSING)

        except json.JSONDecodeError as je:
            self.get_logger().error(f"JSON decode error in listener_callback: {je}")
            self.get_logger().error(f"Received invalid JSON: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in listener_callback: {e}")
            self.get_logger().error(f"Exception type: {type(e).__name__}")

    def request_memory_for_target(self, target_id):
        """Request memory for a specific target"""
        target_state = self.target_manager.get_target(target_id)
        if not target_state:
            self.get_logger().error(f"Cannot request memory for unknown target: {target_id}")
            return
        
        self.get_logger().info(f"Requesting memory for target {target_id}")
        
        if not self.getMemory_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Memory service not available for target {target_id}")
            target_state.set_memory("""{"error":"memory_service_unavailable"}""")
            target_state.set_state(TargetState.PROCESSING)
        else:
            req = Chat.Request()
            req.message = json.dumps({
                "request": target_state.error_message,
                "target": target_id
            })
            future = self.getMemory_client.call_async(req)
            future.add_done_callback(lambda f: self.memory_response_callback(f, target_id))

    def memory_response_callback(self, future, target_id):
        """Handle memory response for a specific target"""
        self.get_logger().info(f"Memory response callback triggered for target {target_id}")
        
        target_state = self.target_manager.get_target(target_id)
        if not target_state:
            self.get_logger().warning(f"Memory response for unknown target: {target_id}")
            return
        
        try:
            memory_response = future.result()
            memory_content = memory_response.response if memory_response else "none"
            
            target_state.set_memory(memory_content)
            target_state.set_state(TargetState.PROCESSING)
            self.get_logger().info(f"Memory content stored for target {target_id}")
                
        except Exception as e:
            self.get_logger().error(f"Memory response error for {target_id}: {e}")
            
            target_state.set_memory("""{"error":"memory_service_error"}""")
            target_state.set_state(TargetState.PROCESSING)

    def process_targets(self):
        """Process targets that are ready for processing directly without creating additional timers"""
        target_ids = self.target_manager.get_all_target_ids()
        
        for target_id in target_ids:
            target_state = self.target_manager.get_target(target_id)
            if not target_state:
                continue 
                
            if (target_state.get_state() != TargetState.PROCESSING or 
                target_state.is_processing()):
                continue
            
            target_state.set_processing(True)
            self.process_error_handling(target_id)


    def process_error_handling(self, target_id):
        """Process error handling for a specific target"""
        target_state = self.target_manager.get_target(target_id)
        if not target_state:
            self.get_logger().error(f"Cannot process error for unknown target: {target_id}")
            return
        
        self.get_logger().info(f"Starting error handling process for target {target_id}")
        
        try:
            # Compose AI prompt messages
            messages = [{"role": "system", "content": f"Instructions: {self.INSTRUCTIONS}"}]
            
            # Include action called information
            if target_state.action_called != "none":
                messages.append({"role": "user", "content": f"Action Called: {target_state.action_called}"})
            
            # Include error message
            if target_state.error_message != "none":
                messages.append({"role": "user", "content": f"Error Message: {target_state.error_message}"})
            
            # Include memory if available
            if target_state.memory != "none":
                messages.append({"role": "user", "content": f"Memory: {target_state.memory}"})
            
            # Include runtime messages if available
            if target_state.runtime_messages != "none":
                messages.append({"role": "user", "content": f"Runtime Messages: {target_state.runtime_messages}"})
            
            # Include UMRF queue if available + Remove first action (ERROR)
            if target_state.umrf_queue != "none":
                queue_data = json.loads(target_state.umrf_queue)
                if isinstance(queue_data, list) and len(queue_data) > 0:
                    queue_data = queue_data[1:]
                queue_str = json.dumps(queue_data)
                queue_summary = queue_str[:1000] + "..." if len(queue_str) > 1000 else queue_str
                messages.append({"role": "user", "content": f"Upcoming Actions Queued: {queue_summary}"})

            # Include additional task data if available
            if target_state.process_data_text != "none":
                messages.append({"role": "user", "content": f"Task Data Text: {target_state.process_data_text}"})
            if target_state.process_data_images != "none":
                messages.append({"role": "user", "content": f"Task Data Images: {target_state.process_data_images}"})
            
            # Store the messages in the target state
            for msg in messages:
                target_state.add_message(msg["role"], msg["content"])
            
            self.get_logger().info(f"Messages prepared for target {target_id}")

            # Call the AI to help resolve the error
            self.get_logger().info(f"Calling AI for target {target_id}")
            
            # Get AI response
            assistant_reply = ai_core.AI_Image_Prompt(
                messages, 
                target_state.auto_temperature, 
                target_state.auto_max_tokens,
                target_state.auto_frequency_penalty, 
                target_state.auto_presence_penalty
            )
            self.get_logger().info(f"AI response received for target {target_id}")
            
            # Process the AI response
            if target_state.process_auto_solve == "true":
                assistant_reply = self.llm_response_parser(assistant_reply)
                
                try:
                    auto_solving_response = json.loads(assistant_reply)
                    success = auto_solving_response.get("success", "false")
                    
                    if success == "true":
                        self.get_logger().info(f"Auto solving succeeded for target {target_id}!")
                        target_state.set_state(TargetState.RESOLVED)
                        
                        # Add target to the response if not present
                        if "target" not in auto_solving_response:
                            auto_solving_response["target"] = target_id
                        
                        # Publish the response to /umrf_correction
                        msg = String()
                        msg.data = json.dumps(auto_solving_response)
                        self.error_queue_update_pub.publish(msg)

                        # Publish the request to /chat_interface_feedback
                        request_msg = String()
                        request_data = {"targets": [target_id], "type": "info", "message": auto_solving_response.get("message", "Error Handling completed autonomously")}
                        request_msg.data = json.dumps(request_data)
                        self.prompt_request_pub.publish(request_msg)
                        
                        # Clean up the target
                        self.target_manager.remove_target(target_id)
                    else:
                        # Log assistant response into target state
                        target_state.add_message(
                            "user", 
                            f"(time: {time.time()}) Original error message: {target_state.error_message}"
                        )
                        target_state.add_message(
                            "assistant", 
                            f"(time: {time.time()}) Autonomous Assistant message using only memory: {json.dumps(auto_solving_response)}"
                        )
                        target_state.error_message = auto_solving_response.get("message", target_state.error_message)
                        target_state.set_processing(False) 

                        self.get_logger().info(f"Auto solving failed for target {target_id}, trying user prompt")
                        self.handle_user_prompt(target_id)
                except json.JSONDecodeError:
                    self.get_logger().error(f"JSON decode error with assistant response for {target_id}")
                    target_state.set_processing(False)  
                    self.handle_user_prompt(target_id)
            else:
                self.get_logger().info(f"Skipping automatic solving for {target_id}, directly prompting user")
                target_state.set_processing(False) 
                self.handle_user_prompt(target_id)
                
        except Exception as e:
            self.get_logger().error(f"Error during AI processing for target {target_id}: {e}")
            # Make sure target exists before trying to update it
            target_state = self.target_manager.get_target(target_id)
            if target_state:
                target_state.set_processing(False)
                self.handle_user_prompt(target_id)

    def handle_user_prompt(self, target_id):
        """Start the user prompt process for a target"""
        self.get_logger().info(f"Starting user prompt handling for target {target_id}")
        
        target_state = self.target_manager.get_target(target_id)
        if not target_state:
            self.get_logger().error(f"Cannot handle user prompt for unknown target: {target_id}")
            return
        
        target_state.trial_nb = 0
        
        if target_state.error_message == "none":
            self.get_logger().error(f"No error message for target {target_id}")
            self.stop_status(target_id)
            return
        
        self.start_user_prompt_trial(target_id)

    def start_user_prompt_trial(self, target_id):
        """Start a user prompt trial for a target"""
        target_state = self.target_manager.get_target(target_id)
        if not target_state:
            self.get_logger().error(f"Cannot start prompt trial for unknown target: {target_id}")
            return
        
        # Increment trial number
        trial_nb = target_state.increment_trial()
        
        if trial_nb > target_state.max_trials:
            self.get_logger().info(f"Max trials reached for target {target_id}. Stopping.")
            target_state.set_state(TargetState.FAILED)
            self.stop_status(target_id)
            return
        
        self.get_logger().info(f"Trial {trial_nb} of {target_state.max_trials} for target {target_id}")
        
        # Save Assistant request
        target_state.add_message(
            "user", 
            f"(time: {time.time()}) Assistant message to user: {target_state.error_message}"
        )
        
        # Mark as waiting for response
        target_state.set_waiting_for_user_response(True)
        target_state.set_state(TargetState.WAITING_FOR_USER)
        
        # Publish the request to /chat_interface_feedback
        request_msg = String()
        request_data = {"targets": [target_id], "type": "request", "message": target_state.error_message}
        request_msg.data = json.dumps(request_data)
        self.prompt_request_pub.publish(request_msg)
        
        self.get_logger().info(f"Request published for target {target_id}, now waiting for user response")

    def prompt_response_callback(self, msg):
        """Handle responses from users via chat interface"""
        self.get_logger().info(f"Received message from chat interface: {msg.data}")
        
        try:
            response_data = json.loads(msg.data)
            response_targets = response_data.get("targets", [])
            response_message = response_data.get("message", "")
            msg_type = response_data.get("type", "")
            
            if msg_type != "response":
                self.get_logger().debug(f"Ignoring non-response message type: {msg_type}")
                return
            
            # Process the response for each target directly
            for target_id in response_targets:
                self.process_target_response(target_id, response_message)
        
        except json.JSONDecodeError as je:
            self.get_logger().error(f"JSON decode error in prompt_response_callback: {je}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in prompt_response_callback: {e}")
    

    def process_target_response(self, target_id, response_message):
        """Process a user response for a specific target"""
        self.get_logger().info(f"Processing response for target {target_id}")
        
        target_state = self.target_manager.get_target(target_id)
        if not target_state:
            self.get_logger().warning(f"Received response for unknown target: {target_id}")
            return
        
        if not target_state.get_waiting_for_user_response():
            self.get_logger().debug(f"Target {target_id} not waiting for response, ignoring")
            return
        
        # Mark as being processed
        target_state.set_processing(True)
        
        # Update state
        target_state.set_waiting_for_user_response(False)
        
        # Add user response to messages
        target_state.add_message(
            "user", 
            f"(time: {time.time()}) User message: {response_message}"
        )
        
        # Process with AI
        try:
            messages = target_state.get_messages_copy()
            temperature = target_state.user_temperature
            max_tokens = target_state.user_max_tokens
            frequency_penalty = target_state.user_frequency_penalty
            presence_penalty = target_state.user_presence_penalty
            
            prompt_reply = ai_core.AI_Image_Prompt(
                messages, temperature, max_tokens, frequency_penalty, presence_penalty
            )
            
            prompt_reply = self.llm_response_parser(prompt_reply)
            
            try:
                prompt_solving = json.loads(prompt_reply)
                success = prompt_solving.get("success", "false")
                system_cmd = prompt_solving.get("system_cmd", [])
                
                if isinstance(success, str):
                    success = success.lower() == "true"
                
                target_state.error_message = prompt_solving.get("message", "")

                # Check for system commands
                if system_cmd and isinstance(system_cmd, list):
                    for cmd_obj in system_cmd:
                        if isinstance(cmd_obj, dict):
                            if "stop" in cmd_obj:
                                self.get_logger().info(f"User requested to stop error handling for {target_id}")
                                target_state.set_state(TargetState.FAILED)

                                # Publish the request to /chat_interface_feedback
                                request_msg = String()
                                request_data = {"targets": [target_id], "type": "info", "message": prompt_solving.get("message", "")}
                                request_msg.data = json.dumps(request_data)
                                self.prompt_request_pub.publish(request_msg)

                                self.stop_status(target_id)
                                return
                            elif "skip" in cmd_obj:
                                self.get_logger().info(f"User requested to skip error handling for {target_id}")
                                target_state.set_state(TargetState.SKIPPED)
                                
                                # Publish the request to /chat_interface_feedback
                                request_msg = String()
                                request_data = {"targets": [target_id], "type": "info", "message": prompt_solving.get("message", "")}
                                request_msg.data = json.dumps(request_data)
                                self.prompt_request_pub.publish(request_msg)

                                self.skip_status(target_id)
                                return
                
                if success:
                    self.get_logger().info(f"User-assisted solving succeeded for target {target_id}!")
                    target_state.set_state(TargetState.RESOLVED)
                    
                    # Add target to the response if not present
                    if "target" not in prompt_solving:
                        prompt_solving["target"] = target_id
                        prompt_reply = json.dumps(prompt_solving)
                    
                    # Publish the request to /chat_interface_feedback
                    request_msg = String()
                    request_data = {"targets": [target_id], "type": "info", "message": prompt_solving.get("message", "")}
                    request_msg.data = json.dumps(request_data)
                    self.prompt_request_pub.publish(request_msg)

                    # Publish new queue to the error handling update topic
                    msg = String()
                    msg.data = prompt_reply
                    self.error_queue_update_pub.publish(msg)
                    
                    # Clean up
                    self.target_manager.remove_target(target_id)
                else:
                    self.get_logger().info(f"User-assisted solving failed for target {target_id}, trying next trial")
                    target_state.set_processing(False)  
                    self.start_user_prompt_trial(target_id)
            
            except json.JSONDecodeError:
                self.get_logger().error(f"JSON decode error with prompt reply for target {target_id}")
                target_state.set_processing(False) 
                self.start_user_prompt_trial(target_id)
        
        except Exception as e:
            self.get_logger().error(f"Error processing user response for target {target_id}: {e}")
            # Check if target still exists
            target_state = self.target_manager.get_target(target_id)
            if target_state:
                target_state.set_processing(False)
                self.start_user_prompt_trial(target_id)
            
    def stop_status(self, target_id):
        """Send stop status for a target and clean up"""
        self.get_logger().info(f"Sending stop status for target {target_id}")
        
        msg = String()
        msg.data = json.dumps({"system_cmd": [{"stop": 0}], "target": target_id})
        self.error_queue_update_pub.publish(msg)
        
        self.target_manager.remove_target(target_id)

    def skip_status(self, target_id):
        """Handle skip status for a target"""
        msg = String()
        msg.data = json.dumps({"system_cmd": [{"skip": 0}], "target": target_id})
        self.error_queue_update_pub.publish(msg)

        self.target_manager.remove_target(target_id)
        
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
        self.get_logger().info("Shutting down error handler node")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = ErrorHandler()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped via keyboard interrupt.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()