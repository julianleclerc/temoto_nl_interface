#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from nl_interface_msgs.srv import Chat
import json
import time
from rclpy.executors import MultiThreadedExecutor

from temoto_msgs.msg import UmrfGraphStart
from temoto_msgs.msg import UmrfGraphStop
from temoto_msgs.msg import UmrfGraphResume
from temoto_msgs.msg import UmrfGraphPause
from temoto_msgs.msg import UmrfGraphModify 
from temoto_msgs.msg import UmrfGraphFeedback

import traceback

class TargetUMRF:
    """Class representing a target with its associated UMRF data and state."""
    
    def __init__(self, target_name, graph_name=None, graph_description=""):
        self.target = target_name
        self.graph_name = graph_name or time.strftime("%Y%m%d%H%M%S")
        self.graph_description = graph_description
        self.queue_json = []
        self.completed_json = []
        self.hold_queue_json = []
        self.stop_status = False
        self.hold_status = False
        self.graph_state = "inactive"
    
    def is_on_hold(self):
        """Check if this target is on hold."""
        return self.hold_status
    
    def add_to_queue(self, queue_items):
        """Add items to the target's queue."""
        self.queue_json.extend(queue_items)
    
    def add_to_hold_queue(self, json_data):
        """Add JSON data to the hold queue for later processing."""
        self.hold_queue_json.append(json_data)
        
    def pop_from_queue(self, index):
        """Remove and return an item from the queue at the specified index."""
        try:
            return self.queue_json.pop(index)
        except IndexError:
            return None
            
    def clear_queue(self):
        """Clear all items from the queue."""
        self.queue_json.clear()
        
    def process_hold_queue(self):
        """Process and clear the hold queue, returning the items that were on hold."""
        hold_queue = self.hold_queue_json.copy()
        self.hold_queue_json.clear()
        return hold_queue
        
    def set_hold(self, status):
        """Set the hold status of this target."""
        self.hold_status = status
        
    def set_stop(self, status):
        """Set the stop status of this target."""
        self.stop_status = status
        
    def set_graph_state(self, state):
        """Set the graph state of this target."""
        self.graph_state = state
        
    def reset_state(self):
        """Reset the target's state and queues."""
        self.graph_state = "inactive"
        self.stop_status = False 
        self.hold_status = False
        self.hold_queue_json = []
        self.completed_json = []
        self.queue_json = []

class TargetManager:
    """Class for managing multiple TargetUMRF instances."""
    
    def __init__(self):
        self.targets = {}  # Dictionary mapping target names to TargetUMRF objects
        
    def get_target(self, target_name):
        """Get a target by name, returning None if it doesn't exist."""
        return self.targets.get(target_name)
        
    def create_target(self, target_name, graph_description=""):
        """Create a new target with the given name and description."""
        if target_name not in self.targets:
            self.targets[target_name] = TargetUMRF(target_name, graph_description=graph_description)
        return self.targets[target_name]
        
    def get_or_create_target(self, target_name, graph_description=""):
        """Get an existing target or create it if it doesn't exist."""
        if target_name not in self.targets:
            return self.create_target(target_name, graph_description)
        return self.targets[target_name]
        
    def target_exists(self, target_name):
        """Check if a target with the given name exists."""
        return target_name in self.targets
        
    def get_all_targets(self):
        """Get a list of all targets."""
        return list(self.targets.values())

class UMRF_PLANNER(Node):
    
    def __init__(self):
        super().__init__('planner_node')
        
        self.target_manager = TargetManager()

        # Define node I/O
        self.chat_action = self.create_subscription(String,'chat_action',self.chat_listener,10) 
        self.error_handling_message = self.create_publisher(String, '/error_handling_message',10) 
        self.error_queue_update = self.create_subscription(String,'/umrf_correction',self.umrf_graph_correction,10)
        self.umrf_feedback_sub = self.create_subscription(UmrfGraphFeedback,'/umrf_graph_feedback',self.umrf_feedback,10)
        self.umrf_feedback_pub = self.create_publisher(UmrfGraphFeedback,'/umrf_graph_feedback',10)
        self.umrf_stop_pub = self.create_publisher(UmrfGraphStop, '/umrf_graph_stop', 10)
        self.umrf_resume_pub = self.create_publisher(UmrfGraphResume, '/umrf_graph_resume', 10)
        self.umrf_pause_pub = self.create_publisher(UmrfGraphPause, '/umrf_graph_pause', 10)
        self.umrf_modify_publisher = self.create_publisher(UmrfGraphModify, '/umrf_graph_modify', 10)
        self.umrf_start_publisher = self.create_publisher(UmrfGraphStart, '/umrf_graph_start', 10)
        self.add_to_memory_pub = self.create_publisher(String,'/add_to_memory',10)
        self.remove_memory_pub = self.create_publisher(String,'/remove_memory',10)


        self.command_queue = {"system_cmd": []}

        self.get_logger().info('PlannerNode is active.')

    #### Queue commands #############
    #
    #   - updating the queue
    #   - adding new element to the queue
    #
    #################################

    def update_json_queue(self, target):
        """
        Serialize the current queue to JSON and publish it.
        """
        ### Transform into UMRF
        # umrf_graph: complete graph of actions
        # init_action_name: name of the next action that will be called
        # init_action_id : id of the next action that will be called
        ####

        self.get_logger().info(f'Updating JSON queue for target: {target}')

        # Get the target umrf data
        target_umrf = self.target_manager.get_target(target)
        if target_umrf is None:
            self.get_logger().error(f'Target "{target}" not found in target manager.')
            return

        # Build proper UMRF
        umrf_graph, resume_action, resume_id = self._build_umrf_graph(target_umrf)

        # Extract data
        graph_name = target_umrf.graph_name
        graph_state = target_umrf.graph_state

            # Publish Modified Graph
        if graph_state == "active":
            self.get_logger().info(f'graph is active, modyfing graph')
            msg = UmrfGraphModify()

            msg.umrf_graph_name = graph_name
            msg.targets = [target]
            msg.modified_graph = json.dumps(umrf_graph)
            msg.continue_from = f"{resume_action}_{resume_id}"

            self.umrf_modify_publisher.publish(msg)
            self.get_logger().info(f'Published: {msg}')
                    
        # Publish Starting Graph
        else: 
            self.get_logger().info(f'graph is inactive, starting graph')
            msg = UmrfGraphStart()

            msg.umrf_graph_name = ""
            msg.name_match_required = False
            msg.targets = [target]
            msg.umrf_graph_json = json.dumps(umrf_graph)

            self.umrf_start_publisher.publish(msg)
            self.get_logger().info(f'Published: {msg}')

            # Update the action engine to active status
            target_umrf.set_graph_state("active")    

    def add_to_queue(self, queue_input_json, target):
        """
        Append items from the input JSON to the target's queue.
        """
        if "queue" in queue_input_json and isinstance(queue_input_json["queue"], list):
            # Get the target
            target_umrf = self.target_manager.get_target(target)
            if target_umrf:
                # check if stopped
                if target_umrf.stop_status:
                    # If target is stopped delete the noncompleted queue
                    target_umrf.clear_queue()
                    target_umrf.add_to_queue(queue_input_json["queue"])
                    self.get_logger().info(f'Added {len(queue_input_json["queue"])} items to queue for target: {target}')
                    self.update_json_queue(target)
                else:
                    # Add new actions to the target's queue
                    target_umrf.add_to_queue(queue_input_json["queue"])
                    self.get_logger().info(f'Added {len(queue_input_json["queue"])} items to queue for target: {target}')
                    
                    if target_umrf.hold_status == False:
                        self.update_json_queue(target)
            else:
                self.get_logger().error(f'Target "{target}" not found in target manager.')
        else:
            self.get_logger().error('"queue" key is missing or not a list in the input JSON')


    #### System commands ############
    #
    #   - "pop" element from queue
    #   - "clear" queue -> clear and stop
    #   - "stop" process -> pauses
    #   - "start" process -> resumes
    #   - "skip" action -> pauses, pops, resumes
    #   - "add_memory" -> adds to memory
    #
    #################################
        
    def add_memory(self, value, target):
        '''
        Add to memory
        '''
        try:
            # Check if value is a dictionary
            if isinstance(value, dict):
                memory_data = value
            else:
                # Try to parse if it's a string
                try:
                    memory_data = json.loads(value)
                except (json.JSONDecodeError, TypeError):
                    self.get_logger().error(f'Invalid memory data format: {value}')
                    return
                    
            # Check if required fields exist
            if "data" not in memory_data or "info" not in memory_data:
                self.get_logger().error(f'Missing required fields (data or info) in memory data')
                return
                
            # Prepare memory message
            memory = {
                f"""memory_{time.strftime("%Y%m%d%H%M%S")}""": {
                    "type": "text",
                    "data": memory_data["data"],
                    "info": memory_data["info"]
                }
            }

            # Create message
            msg = String()
            msg.data = json.dumps(memory)
            self.get_logger().info(f'Publishing memory: {msg.data}')
            self.add_to_memory_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error adding memory: {e}')
            return        


    def pop(self, index, target):
        '''
        Pops a unique element from the target's queue.
        '''
        # Get the target
        target_umrf = self.target_manager.get_target(target)
        
        if target_umrf is None:
            self.get_logger().error(f'Target "{target}" not found in target manager.')
            return None

        try:
            # Pop the element at the given index from the target's queue
            popped_action = target_umrf.pop_from_queue(index)
            self.update_json_queue(target)
            self.get_logger().info(f"Popped action at index {index} for target '{target}': {popped_action}")
            return popped_action
        except IndexError:
            self.get_logger().error("Attempted to pop from an empty queue or index out of range.")
            return None
        except TypeError:
            self.get_logger().error("Index must be an integer.")
            return None

    def clear(self, value, target):
        '''
        Clear and Stops the JSON queue from any current actions for the given target.
        '''
        # Get the target
        target_umrf = self.target_manager.get_target(target)
        if target_umrf is None:
            self.get_logger().error(f'Target "{target}" not found in target manager.')
            return

        # Update statuses for this target
        target_umrf.reset_state()

        # Update overall status and publish stop signal
        msg = UmrfGraphStop()
        msg.umrf_graph_name = target_umrf.graph_name
        msg.targets = [target]
        self.umrf_stop_pub.publish(msg)

        self.get_logger().info(f"Actions stopped for target: {target}")


    def start(self, value, target):
        '''
        Resume queue to start action for the given target.
        '''
        # Get the target
        target_umrf = self.target_manager.get_target(target)
        if target_umrf is None:
            self.get_logger().error(f'Target "{target}" not found in target manager.')
            return

        # Update the statuses for this target
        target_umrf.set_stop(False)
        target_umrf.set_hold(False)

        # Update overall status and publish stop signal
        msg = UmrfGraphResume()
        msg.umrf_graph_name = target_umrf.graph_name
        msg.targets = [target]
        self.umrf_resume_pub.publish(msg)

        # Reflect the changes in the JSON queue
        self.update_json_queue(target)
        self.get_logger().info(f"Actions restarted for target: {target}")

    def stop(self, value, target):
        '''
        Pause ongoing action for the given target.
        '''
        # Get the target
        target_umrf = self.target_manager.get_target(target)
        
        if target_umrf is None:
            self.get_logger().error(f'Target "{target}" not found in target manager.')
            return

        # Update statuses for this target
        target_umrf.set_stop(True)

        # Update overall status and publish stop signal
        msg = UmrfGraphPause()
        msg.umrf_graph_name = target_umrf.graph_name
        msg.targets = [target]
        self.umrf_pause_pub.publish(msg)

        self.get_logger().info(f"Actions paused for target: {target}")

    def skip(self, value, target):
        '''
        skip ongoing action
        '''
        self.stop(0, target)
        self.pop(0, target)
        self.start(0, target)
        self.get_logger().info("Action Skipped")


    def replace_action(self, index, queue_input_json, target):
        # Get the target
        target_umrf = self.target_manager.get_target(target)
        
        if target_umrf is None:
            self.get_logger().error(f'Target "{target}" not found in target manager.')
            return

        # Remove the specified element from the queue
        target_umrf.pop_from_queue(index)
        
        # Insert the elements from the input queue at the specified index
        for i, element in enumerate(queue_input_json["queue"]):
            target_umrf.queue_json.insert(index + i, element)
            
        # Update the JSON queue
        self.update_json_queue(target)

    def _call_action(self, action_name, action_value, target):
        """
        Dynamically calls a method based on the action_name.
        """
        method = getattr(self, action_name, None)
        if callable(method):
            if target:
                method(action_value, target)
            else:
                method(action_value)
        else:
            self.get_logger().info(f"No method found for action: {action_name}")

    def system_commands(self, system_input_json, target):
        if "system_cmd" in system_input_json and isinstance(system_input_json["system_cmd"], list):
            self.command_queue["system_cmd"].extend(system_input_json["system_cmd"])
            self.get_logger().info(f'Added {len(system_input_json["system_cmd"])} items to system_cmd')
        else:
            self.get_logger().error('"system_cmd" key is missing or not a list in the input JSON')

        for command in self.command_queue.get("system_cmd", []):
            if isinstance(command, dict):
                for command_name, command_value in command.items():
                    if target:
                        self._call_action(command_name, command_value, target)
                    else:
                        self._call_action(command_name, command_value)
            else:
                self.get_logger().error(f"Invalid command format: {command}")

        # Clear the command_queue after processing
        self.command_queue["system_cmd"].clear()

    #### Chat callback ##############
    #
    # Listens to chat_action topic and handles operations
    #
    #################################

    def is_empty(self, value):
        if value is None:  # Check for null
            return True
        if isinstance(value, str) and value.strip() == "":
            return True
        if isinstance(value, (list, dict)) and len(value) == 0:  # Empty list or dictionary
            return True
        return False

    def chat_listener(self, msg):
        """
        Callback function that gets called when a message is received on 'chat_action' topic.
        """
        self.get_logger().info(f'I heard: "{msg.data}"')

        try:
            # Parse the incoming JSON message
            json_response = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}')
            return
        
        for target in json_response.get("targets", []):

            # Get or create the target
            target_umrf = self.target_manager.get_or_create_target(
                target, 
                graph_description=time.strftime("%Y%m%d%H%M%S")
            )
            
            # Check if this target is on hold
            is_on_hold = target_umrf.is_on_hold()
            has_unhold_cmd = self._has_unhold_command(json_response.get("system_cmd", []))
            
            # Process unhold commands immediately even for on-hold targets
            if is_on_hold and has_unhold_cmd:
                if "system_cmd" in json_response and not self.is_empty(json_response["system_cmd"]):
                    # Extract just the unhold command
                    unhold_cmds = [cmd for cmd in json_response.get("system_cmd", []) 
                                if isinstance(cmd, dict) and "unhold" in cmd]
                    if unhold_cmds:
                        system_input_json = {"system_cmd": unhold_cmds}
                        self.system_commands(system_input_json, target)
                continue
                
            if is_on_hold:
                # Target is on hold - add to hold queue instead of processing
                target_umrf.add_to_hold_queue(json_response)
                self.get_logger().info(f'Target {target} is on hold, actions queued for later processing')
                continue

            # Normal processing for targets not on hold
            # Process system commands for this target
            if "system_cmd" in json_response and not self.is_empty(json_response["system_cmd"]):
                system_input_json = {"system_cmd": json_response["system_cmd"]}
                self.system_commands(system_input_json, target)
            
            # Process queue updates for this target
            if "queue" in json_response and not self.is_empty(json_response["queue"]):
                queue_input_json = {"queue": json_response["queue"]}
                self.add_to_queue(queue_input_json, target)

    #### ERROR HANDLING DEF ##############
    #
    # ABILITY TO HOLD/UNHOLD OPERATIONS
    # CHECK IF A TARGET IS ON HOLD
    # MODIFY THE GRAPH IN RESCPECT ERROR HANDLING
    #
    #################################

    def hold(self, value, target):
        '''
        Hold ongoing action for the given target.
        '''
        # Get the target
        target_umrf = self.target_manager.get_target(target)
        
        if target_umrf is None:
            self.get_logger().error(f'Target "{target}" not found in target manager.')
            return
            
        # Update hold status for this target
        target_umrf.set_hold(True)
        
        # Update overall system status
        self.get_logger().info(f"Planning hold for process for target: {target}")

    def unhold(self, value, target):
        '''
        Unhold ongoing action for the given target and process any queued actions.
        '''
        # Get the target
        target_umrf = self.target_manager.get_target(target)
        
        if target_umrf is None:
            self.get_logger().error(f'Target "{target}" not found in target manager.')
            return
        
        # Process any actions that were queued while the target was on hold
        hold_queue = target_umrf.process_hold_queue()
        if hold_queue:
            self.get_logger().info(f"Processing {len(hold_queue)} queued actions for target {target}")
            
            # Process each queued action in the order they were received
            for queued_json in hold_queue:
                # Process system commands
                if "system_cmd" in queued_json and not self.is_empty(queued_json["system_cmd"]):
                    system_input_json = {"system_cmd": queued_json["system_cmd"]}
                    self.system_commands(system_input_json, target)
                
                # Process queue updates
                if "queue" in queued_json and not self.is_empty(queued_json["queue"]):
                    queue_input_json = {"queue": queued_json["queue"]}
                    self.add_to_queue(queue_input_json, target)
                
                # If nothing is provided, skip this
                if ("system_cmd" not in queued_json or self.is_empty(queued_json["system_cmd"])) and \
                ("queue" not in queued_json or self.is_empty(queued_json["queue"])):
                    self.skip(0, target)
            
            # Update hold status for this target only
            target_umrf.set_hold(False)
            self.get_logger().info(f"Planning unhold for target: {target}")

        else:
            self.get_logger().info(f"Not actions on hold for: {target}")
            target_umrf.set_hold(False)
            
    def _has_unhold_command(self, system_cmds):
        """
        Check if the system commands include an unhold command.
        """
        if not system_cmds:
            return False
            
        for cmd in system_cmds:
            if isinstance(cmd, dict) and "unhold" in cmd:
                return True
        return False

    def umrf_graph_correction(self, msg):
        self.get_logger().info(f'I heard planning update: "{msg.data}"')
        try:
            # Parse the incoming JSON message
            json_response = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}')
            return

        self.get_logger().debug(f'Parsed JSON response: {json_response}')

        # Get the target from the JSON if available, otherwise use the first target in the queue
        target = json_response.get("target")
        if not target:
            # Get the first target in the manager if any
            targets = self.target_manager.get_all_targets()
            if targets:
                target = targets[0].target
        
        if not target:
            self.get_logger().error("No target specified and no targets in queue")
            return

        # Check and process the "queue" key 
        if "queue" in json_response and not self.is_empty(json_response["queue"]):
            queue_input_json = {"queue": json_response["queue"]}
            self.replace_action(0, queue_input_json, target)
        else:
            self.get_logger().warning('No "queue" key found or "queue" is empty in the received JSON.')

        # Unhold the target
        self.unhold(0, target)

        # Check and process the "system_cmd" key - here used to stop the graph
        if "system_cmd" in json_response and not self.is_empty(json_response["system_cmd"]):
            system_input_json = {"system_cmd": json_response["system_cmd"]}
            self.system_commands(system_input_json, target)
        else:
            self.get_logger().warning('No "system_cmd" key found or "system_cmd" is empty in the received JSON, resuming process.')



    #### UMRF FEEDBACK callback ##############
    #
    # Listens to the umrf feedback everytime an action is completed in action engine
    # Decomposes graph into two: completed, queued
    # Handles for calling out errors
    #
    #################################

    def umrf_feedback(self, msg):
        self.get_logger().info("------- UMRF FEEDBACK START -------")
        self.get_logger().info(f"Received UMRF feedback for actor: {msg.actor}")
        self.get_logger().debug(f"Message type: {type(msg).__name__}")
        self.get_logger().debug(f"History entries: {len(msg.history) if msg.history else 0}")

        target = msg.actor
        if not target:
            self.get_logger().error("No target in feedback message")
            return

        if not msg.history or len(msg.history) == 0:
            self.get_logger().warning("Empty history in feedback message")
            return

        try:
            self.get_logger().debug("Attempting to parse most recent JSON entry")
            most_recent_json = msg.history[0]
            self.get_logger().debug(f"JSON string length: {len(most_recent_json)}")
            umrf_history = json.loads(most_recent_json)
            self.get_logger().debug(f"Parsed JSON keys: {list(umrf_history.keys())}")
            self.get_logger().debug(f"Actions count: {len(umrf_history.get('actions', []))}")
        except Exception as e: 
            self.get_logger().error(f"Error fetching latest actions in umrf_feedback: {str(e)}")
            self.get_logger().error(f"Exception type: {type(e).__name__}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            if 'target' in locals():
                self.get_logger().info(f"Stopping processing for target: {target}")
                self.stop(0, target)
                return

        self.get_logger().info(f"Most recent json: {most_recent_json}")

        # Reset Target upon completion or stopped
        try:
            graph_state = umrf_history.get("graph_state")
            self.get_logger().info(f"Graph State: {graph_state}")

            if graph_state in ["FINISHED", "STOPPED"]:
                self.get_logger().debug(f"Graph state indicates completion: {graph_state}")
                target_umrf = self.target_manager.get_target(target)
                self.get_logger().debug(f"Found target in manager: {target_umrf is not None}")
                
                if target_umrf:
                    self.get_logger().debug(f"Current graph state: {target_umrf.graph_state}")
                    self.get_logger().debug(f"Stop status: {target_umrf.stop_status}")
                    self.get_logger().debug(f"Hold status: {target_umrf.hold_status}")
                    
                    # Reset the target state
                    target_umrf.reset_state()
                    
                    # Log the reset
                    self.get_logger().info(f"Reset UMRF graph parameters for target: {target}")
                    self.get_logger().debug("All queue arrays reset to empty")
                    return
        except Exception as e: 
            self.get_logger().error(f"Error processing action state in umrf_feedback: {str(e)}")
            self.get_logger().error(f"Exception type: {type(e).__name__}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            if 'target' in locals():
                self.get_logger().info(f"Stopping processing for target: {target} due to error")
                self.stop(0, target)
                return

        ### REBUILD COMPLETED AND QUEUE ACTIONS BASED ON FEEDBACK
        try:            
            self.get_logger().debug("Starting to process completed and queued actions")    
            
            # Get the target from the manager
            target_umrf = self.target_manager.get_target(target)
                    
            if target_umrf is None:
                self.get_logger().info(f"No target found for: {target}")
                return
            
            # Store the original actions before processing
            original_completed = target_umrf.completed_json.copy()
            original_queue = target_umrf.queue_json.copy()
            original_total_actions = len(original_completed) + len(original_queue)
            
            self.get_logger().debug(f"Original state - completed: {len(original_completed)}, queue: {len(original_queue)}")
            
            # Create a map of action states from the feedback
            action_states = {}
            
            if 'actions' in umrf_history:
                sorted_actions = sorted(umrf_history.get('actions', []), key=lambda x: x.get('instance_id', float('inf')))
                
                for action in sorted_actions:
                    action_name = action.get('name')
                    instance_id = action.get('instance_id')
                    state = action.get('state')
                    
                    action_states[instance_id] = {
                        'name': action_name,
                        'state': state
                    }
                    
                    self.get_logger().debug(f"Action {action_name}_{instance_id}: State={state}")
            
            # Find the dividing line between completed and non-completed actions
            last_completed_id = -1
            for instance_id in sorted(action_states.keys()):
                if action_states[instance_id]['state'] == 'FINISHED':
                    last_completed_id = max(last_completed_id, instance_id)
            
            # Now rebuild the queues based on the last completed ID
            new_completed_json = []
            new_queue_json = []
            
            # Process each action from the original lists
            original_actions = original_completed + original_queue
            
            if last_completed_id >= 0:
                # Move completed actions
                for i in range(last_completed_id + 1):
                    if i < len(original_actions):
                        new_completed_json.append(original_actions[i])
                
                # Move remaining actions to queue
                for i in range(last_completed_id + 1, len(original_actions)):
                    if i < len(original_actions):
                        new_queue_json.append(original_actions[i])
            else:
                # No completed actions, everything stays in queue
                new_completed_json = []
                new_queue_json = original_actions
            
            # Update the target's queues
            target_umrf.completed_json = new_completed_json
            target_umrf.queue_json = new_queue_json
            
            # Verify we haven't lost any actions
            final_total_actions = len(target_umrf.completed_json) + len(target_umrf.queue_json)
            
            if final_total_actions != original_total_actions:
                self.get_logger().error(f"Action count mismatch! Original: {original_total_actions}, Current: {final_total_actions}")
                # Restore original state if we lost actions
                target_umrf.completed_json = original_completed
                target_umrf.queue_json = original_queue
                self.get_logger().error(f"Restored original queue state to prevent action loss")
            else:
                self.get_logger().info(f"Rebuilt queues - completed: {len(target_umrf.completed_json)}, queue: {len(target_umrf.queue_json)} for target: {target}")
        
        except Exception as e: 
            self.get_logger().error(f"Error processing actions in umrf_feedback: {str(e)}")
            self.get_logger().error(f"Exception type: {type(e).__name__}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            if 'target' in locals():
                self.stop(0, target)

        ### ERROR HANDLING
        self.get_logger().info(f"Starting error handling process")
        try:
            # Check for an error in the action immediately following the last completed action
            error_state = False
            error_message = ""
            error_action = None
            error_data = {}

            if 'actions' in umrf_history:
                self.get_logger().debug("Checking for errors in all actions")
                
                for i, action in enumerate(umrf_history.get('actions', [])):
                    current_id = action.get('instance_id')
                    current_state = action.get('state')
                    self.get_logger().debug(f"Examining action {i}: ID={current_id}, State={current_state}")
                    
                    if current_state == "ERROR":
                        error_state = True
                        self.get_logger().info(f"Error detected in action: ID={current_id}, Name={action.get('name')}")
                        
                        # Extract error information from runtime messages
                        if 'runtime' in action and 'messages' in action['runtime']:
                            messages = action['runtime']['messages']
                            self.get_logger().debug(f"Found {len(messages)} runtime messages")
                            
                            if messages:
                                # Sort messages by timestamp
                                sorted_messages = sorted(messages, key=lambda x: x.get('timestamp', 0))
                                self.get_logger().debug(f"Sorted messages by timestamp")
                                
                                # Get the latest message
                                latest_message = sorted_messages[-1].get('message', '')
                                self.get_logger().debug(f"Latest message: {latest_message}")
                                
                                # Try to parse the JSON from the message string
                                try:
                                    # The message is a string containing JSON
                                    message_data = json.loads(latest_message)
                                    self.get_logger().debug(f"Parsed message data keys: {list(message_data.keys())}")
                                    
                                    # Extract the actual error message
                                    error_message = message_data.get('message', '')
                                    self.get_logger().debug(f"Extracted error message: {error_message}")
                                    
                                    # Extract additional data that might be useful
                                    if 'process_instrutions' in message_data:
                                        error_data['process_instrutions'] = message_data['process_instrutions']
                                        self.get_logger().debug("Added process_instrutions to error data")
                                    if 'process_data_text' in message_data:
                                        error_data['process_data_text'] = message_data['process_data_text']
                                        self.get_logger().debug("Added process_data_text to error data")
                                    if 'process_data_images' in message_data:
                                        error_data['process_data_images'] = message_data['process_data_images']
                                        self.get_logger().debug("Added process_data_images to error data")
                                except json.JSONDecodeError as json_err:
                                    # If JSON parsing fails, use the raw message
                                    self.get_logger().warning(f"JSON decode error: {str(json_err)}")
                                    error_message = latest_message
                                    self.get_logger().debug(f"Using raw message as error message")
            
            if error_state:
                self.get_logger().info(f"Preparing error handling for detected error")
                # Get the target from the manager
                target_umrf = self.target_manager.get_target(target)
                
                if target_umrf is not None and len(target_umrf.queue_json) > 0:
                    error_action = target_umrf.queue_json[0]
                    self.get_logger().debug(f"Found error action: {list(error_action.keys()) if isinstance(error_action, dict) else 'non-dict type'}")
                else:
                    error_action = None
                    self.get_logger().debug("No error action found in queue")

                # Prepare the message for error handling
                self.get_logger().info(f"Preparing error handling message")
                error_handling_message = {
                    "error_message": error_message,
                    "action_called": error_action, 
                    "umrf_queue": json.dumps(target_umrf.queue_json) if target_umrf else "[]",
                    "target": target
                }
                self.get_logger().debug(f"Basic error message prepared")                
                
                # Add error data if available
                if error_data:
                    error_handling_message.update(error_data)
                    self.get_logger().debug(f"Added additional error data: {list(error_data.keys())}")

                error_handling_msg = String()
                error_handling_msg.data = json.dumps(error_handling_message)
                self.get_logger().debug(f"Serialized error handling message, length: {len(error_handling_msg.data)}")

                # Update error handling target state
                self.get_logger().info(f"Putting target {target} on hold due to error")
                self.hold(0, target)

                # Send to error handling - fixed publisher reference
                self.get_logger().info(f"Publishing error handling message")
                self.error_handling_message.publish(error_handling_msg)
                self.get_logger().info(f"Published error message for target {target}")
            else:
                self.get_logger().debug("No error state detected, skipping error handling")
                    
        except Exception as e: 
            self.get_logger().error(f"Error handling error in umrf_feedback: {str(e)}")
            self.get_logger().error(f"Exception type: {type(e).__name__}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            if 'target' in locals():
                self.get_logger().info(f"Stopping processing for target: {target} due to error in error handling")
                self.stop(0, target)
            
            self.get_logger().info("------- UMRF FEEDBACK END -------")

    #### BUILD UMRF ##############
    # 
    # Builds the UMRF Graph in order to publish to the temoto action engine
    #
    #################################

    def _build_umrf_graph(self, target_umrf):
        """
        Build and return the UMRF graph, resume action, and resume ID for the given target.
        The graph includes both completed actions and current queue actions in chronological order.

        Args:
            target_umrf (TargetUMRF): The target UMRF object containing graph information.
            
        Returns:
            tuple: (umrf_graph, resume_action, resume_id) where:
                - umrf_graph is the constructed UMRF graph
                - resume_action is the name of the first action in current_queue
                - resume_id is the instance ID of the first action in current_queue
        """    
        if target_umrf is None:
            self.get_logger().error('Target UMRF is None.')
            return None, None, None
        
        # Use target-specific values
        graph_name = target_umrf.graph_name
        graph_description = target_umrf.graph_description

        # Get both queues
        current_queue = target_umrf.queue_json
        completed_queue = target_umrf.completed_json
        
        self.get_logger().debug(f'Building graph: completed={len(completed_queue)}, queue={len(current_queue)}')
        
        # Track the resume action and its ID
        resume_action = None
        resume_id = None
        
        # If both queues are empty, return empty graph
        if not current_queue and not completed_queue:
            self.get_logger().warning('Both current and completed queues are empty.')
            return {"graph_name": graph_name, "graph_description": graph_description, "actions": []}, None, 0
        
        # Create a linear flow of actions
        actions = []
        instance_id = 0
        
        # Process completed queue first
        for step_idx, step in enumerate(completed_queue):
            for action_name, action_data in step.items():
                # Create the action entry
                action_info = {
                    "name": action_name,
                    "instance_id": instance_id,
                    "type": "sync"
                }
                
                # Handle parameters
                if isinstance(action_data, dict):
                    if "input_parameters" in action_data:
                        action_info["input_parameters"] = action_data["input_parameters"]
                    
                    if "output_parameters" in action_data:
                        action_info["output_parameters"] = action_data["output_parameters"]
                
                # Add parent/child relationships for linear flow
                if instance_id > 0:
                    action_info.setdefault('parents', [])
                    action_info['parents'].append({
                        "name": actions[-1]["name"],
                        "instance_id": instance_id - 1,
                        "required": True,
                        "conditions": [
                            "on_true -> run",
                            "on_false -> run",
                            "on_stopped -> ignore",
                            "on_error -> ignore"
                        ]
                    })
                    
                    # Add the current action as a child to the previous action
                    actions[-1].setdefault('children', [])
                    actions[-1]['children'].append({
                        "name": action_name,
                        "instance_id": instance_id
                    })
                
                actions.append(action_info)
                instance_id += 1
        
        # Now process current queue
        for step_idx, step in enumerate(current_queue):
            for action_name, action_data in step.items():
                # Create the action entry
                action_info = {
                    "name": action_name,
                    "instance_id": instance_id,
                    "type": "sync"
                }
                
                # If this is the first action in the current queue, set it as the resume point
                if resume_action is None:
                    resume_action = action_name
                    resume_id = instance_id
                
                # Handle parameters
                if isinstance(action_data, dict):
                    if "input_parameters" in action_data:
                        action_info["input_parameters"] = action_data["input_parameters"]
                    
                    if "output_parameters" in action_data:
                        action_info["output_parameters"] = action_data["output_parameters"]
                
                # Add parent/child relationships for linear flow
                if instance_id > 0:
                    action_info.setdefault('parents', [])
                    action_info['parents'].append({
                        "name": actions[-1]["name"],
                        "instance_id": instance_id - 1,
                        "required": True,
                        "conditions": [
                            "on_true -> run",
                            "on_false -> run",
                            "on_stopped -> ignore",
                            "on_error -> ignore"
                        ]
                    })
                    
                    # Add the current action as a child to the previous action
                    actions[-1].setdefault('children', [])
                    actions[-1]['children'].append({
                        "name": action_name,
                        "instance_id": instance_id
                    })
                
                actions.append(action_info)
                instance_id += 1
        
        # Handle edge case: if no resume point was set but we have actions
        if resume_action is None:
            if current_queue:
                # Find the first action from current_queue in the actions list
                # This should be at index len(completed_queue)
                if len(actions) > len(completed_queue):
                    resume_action = actions[len(completed_queue)]["name"]
                    resume_id = actions[len(completed_queue)]["instance_id"]
                else:
                    # Fall back to the first action
                    resume_action = actions[0]["name"]
                    resume_id = 0
            elif actions:
                # All actions are completed, use the last action
                resume_action = actions[-1]["name"]
                resume_id = actions[-1]["instance_id"]
            else:
                # No actions at all
                self.get_logger().error('No actions available in the graph')
                return None, None, None
        
        # Set the entry and exit points of the graph
        graph_entry = []
        graph_exit = []
        
        if actions:
            graph_entry = [{"name": actions[0]["name"], "instance_id": 0}]
            
            # Add conditions to graph_exit
            graph_exit = [
                {
                    "name": actions[-1]["name"],
                    "instance_id": len(actions) - 1,
                    "conditions": [
                        "on_true -> ignore",
                        "on_false -> ignore",
                        "on_error -> ignore"
                    ]
                }
            ]
        
        umrf_graph = {
            "graph_name":  graph_name,
            "graph_description": graph_description,
            "graph_entry": graph_entry,
            "graph_exit": graph_exit,
            "actions": actions
        }
        
        self.get_logger().debug(f"Built UMRF graph with {len(actions)} actions, resume_action={resume_action}, resume_id={resume_id}")
        
        # Additional debug logging for troubleshooting
        if resume_action and resume_id is not None:
            action_found = False
            for action in actions:
                if action["name"] == resume_action and action["instance_id"] == resume_id:
                    action_found = True
                    break
            
            if not action_found:
                self.get_logger().error(f"Resume action {resume_action}_{resume_id} not found in graph")
        
        return umrf_graph, resume_action, resume_id

def main(args=None):
    rclpy.init(args=args)
    planner_node = UMRF_PLANNER()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(planner_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        planner_node.get_logger().info('PlannerNode has been shut down.')
    finally:
        planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()