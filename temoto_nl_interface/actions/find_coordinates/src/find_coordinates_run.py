import rclpy
from rclpy.node import Node
from interfaces.srv import Chat
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

import time
import json
import os
import tf2_ros
import tf_transformations
from geometry_msgs.msg import TransformStamped
import matplotlib.pyplot as plt
from PIL import Image 

import cv2
import numpy as np
import yaml
import random
import math

import base64
import logging
import subprocess
import threading

from chat_service.ai import ai_core
from chat_service.ai.prompts import getcoord_prompt

# Map generation scripts
from chat_service.scripts.getcoord import getcoord_costmap_generation
from chat_service.scripts.getcoord import getcoord_newcoordmap_generation
from chat_service.scripts.getcoord import getcoord_objectmap_generation
from chat_service.scripts.getcoord import getcoord_robotmap_generation
from chat_service.scripts.getcoord import getcoord_scalemap_generation
from chat_service.scripts.getcoord import getcoord_origincoord_return
from chat_service.scripts.getcoord import getcoord_nonTraversable_generation
from chat_service.scripts.getcoord import getcoord_grid_generation
from chat_service.scripts.getcoord import getcoord_pixelcoord_return

# Alternative until a ai gets better
from chat_service.scripts import getcoord_pathfind_return
from chat_service.ai.prompts import getcoord_prompt_id
from chat_service.ai.prompts import getcoord_prompt_coord
from chat_service.ai.prompts import getcoord_prompt_check


class GetCoordNode(Node):
    def __init__(self):
        """
        Initiate GetCoordNode:
            
            - Seperate Node into 2 callbacks to avoid blocking
            - Initialise service server: getCoord
            - Initialise 2 topics: prompt_request (publisher), prompt_response (subscriber)
            - ...
            
        """
        super().__init__('get_coord_service')
        self.get_logger().info('getCoord initialising')

        # Define inflation radius and scale factor
        self.inflation_radius_m = 0.2
        self.scale_factor = 2  # Adjust as needed

        # Create separate callback groups for service and subscriber
        self.service_callback_group = ReentrantCallbackGroup()
        self.subscriber_callback_group = ReentrantCallbackGroup()

        # Initialize threading event and response storage
        self.response_event = threading.Event()
        self.response = None

        # Initialize getCoord service (with own callback group to handle multithread)
        self.srv = self.create_service(Chat,'getCoord',self.get_coord_callback,callback_group=self.service_callback_group)

        # Initialize prompt_request and prompt_response topics (own callback group for the response)
        self.prompt_request_pub = self.create_publisher(String, '/prompt_request', 10)
        self.prompt_response_sub = self.create_subscription(String,'/prompt_response',self.prompt_response_callback,10,callback_group=self.subscriber_callback_group)
        
        # Initialize prompt_info_pub informing user of getcoord successful result
        self.prompt_info_pub = self.create_publisher(String, '/prompt_info', 10)

        # Initialize tf buffer and listener for getting robot's pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Define data directory
        home_dir = os.path.expanduser("~/thesis")
        self.data_dir = os.path.join(home_dir, "data")

        self.get_logger().info('GetCoord Service is ready.')


    def get_coord_callback(self, request, response):
        """
        Sevice callback function to handle coordinate requests.

        Logic:
            - Update object list and map
            - Scale map 
            - Create cost map
            - Get robot coordinates and place on map
            - Populate map with objects
            - Request for coordinates
            - Display new coordinates on map
            - Scale coordinates back to real world reference
            - return new coordinates

        Parameters:
            request: The incoming request containing a JSON message:
            response: Json message containing coordinates and info.
        """

        ### Load data ###
        success = True
        
        try:
            # Load items.json
            workspace_dir = os.getenv('ROS_WORKSPACE', '~/thesis')
            workspace_dir = os.path.expanduser(workspace_dir)
            json_file_path = os.path.join(workspace_dir, 'data', 'items.json')
            with open(json_file_path, 'r') as f:
                self.items_data = json.load(f)

            # Load coordinates data from JSON file
            with open(json_file_path, 'r') as json_file:
                self.data = json.load(json_file)
                self.get_logger().info('Item list loaded')

            # Import AI Search instructions
            self.INSTRUCTIONS = getcoord_prompt.getprompt()
            self.get_logger().debug(f'Instructions Loaded')

        except Exception as e:
            self.get_logger().error(f'Error loading data: {e}')
        
        
        ### Build object maps ###

        try:
            # Parse the incoming JSON message
            request_msg = json.loads(request.message)
            self.get_logger().info(f'Received message: {request_msg}')
            
            # Reset parameters
            self.map_data = "" 
            self.json_map_data_generation() # simplified .json with only ids and descriptions
            self.angle_deg = 0
            
            #### Curently using launching ros2 process to save map and meta data -> ideally receive it through topics ##########

            # Save SLAM image using ROS2 map_saver_cli
            map_prefix = os.path.join(self.data_dir, "map")
            nav2_cmd = f'ros2 run nav2_map_server map_saver_cli -f {map_prefix}'
            self.get_logger().info(f'Executing command: {nav2_cmd}')
            # Uncomment the following line to execute the command
            # subprocess.run(nav2_cmd, shell=True, check=True)

            # Load the saved map image
            map_pgm_path = map_prefix + '.pgm'
            self.get_logger().info(f'Loading map image from {map_pgm_path}')
            map_img = cv2.imread(map_pgm_path, cv2.IMREAD_GRAYSCALE)
            if map_img is None:
                raise FileNotFoundError(f'Failed to load map image from {map_pgm_path}')

            # Load map metadata from YAML file
            map_yaml_path = os.path.join(self.data_dir, 'map.yaml')
            with open(map_yaml_path, 'r') as file:
                map_metadata = yaml.safe_load(file)
                self.resolution = map_metadata.get('resolution', 0.05)      # Get resolution
                self.origin = map_metadata.get('origin', [0.0, 0.0, 0.0])   # Get origin point
                self.get_logger().debug(f'origin: {self.origin}')

            #####################################################################################################################

            # Get Robot position (setting position at origin otherwise)
            try:
                self.trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            except Exception as e:
                self.trans = TransformStamped()
                self.trans.header.frame_id = 'map'
                self.trans.child_frame_id = 'base_link'
                self.trans.transform.translation.x = 0.0
                self.trans.transform.translation.y = 0.0
                self.trans.transform.translation.z = 0.0
                self.trans.transform.rotation.x = 0.0
                self.trans.transform.rotation.y = 0.0
                self.trans.transform.rotation.z = 0.0
                self.trans.transform.rotation.w = 1.0
                self.get_logger().warn(f'Could not get transform: {e}')

            # Apply scale factor to image
            scaled_img, self.resolution = getcoord_scalemap_generation.get(map_img, self.resolution, self.scale_factor)

            # Generate a cost map from the scaled map image
            cost_map = getcoord_costmap_generation.get(scaled_img, self.resolution, self.inflation_radius_m)
            cost_map_path = os.path.join(self.data_dir, 'cost_map.pgm')
            cv2.imwrite(cost_map_path, cost_map)
            self.get_logger().info(f'Saved cost map image to {cost_map_path}')

            cost_map = cv2.cvtColor(cost_map, cv2.COLOR_GRAY2BGR)

            # Darken non traversable areas
            cost_map = getcoord_nonTraversable_generation.get(cost_map, self.resolution, self.origin, self.trans)
            nonTravesable_path = os.path.join(self.data_dir, 'nonTravesable.png')
            cv2.imwrite(nonTravesable_path, cost_map)
            self.get_logger().info(f'Saved cost map image to {nonTravesable_path}')

            # Create a Grid
            self.grid_scale = 20 # in px
            cost_map = getcoord_grid_generation.get(cost_map, self.resolution, self.grid_scale)
            grid_path = os.path.join(self.data_dir, 'grid_map.png')
            cv2.imwrite(grid_path, cost_map)
            self.get_logger().info(f'Saved cost map image to {grid_path}')

            # Populate map cost with objects using items.json
            object_map = getcoord_objectmap_generation.get(cost_map, self.resolution, self.origin, self.items_data)
            object_map_path = os.path.join(self.data_dir, 'object_map.png')  # Use .png extension
            cv2.imwrite(object_map_path, object_map)
            self.get_logger().info(f'Saved object map image to {object_map_path}')
            
            # Add robot to the object map
            robot_map = getcoord_robotmap_generation.get(object_map, self.resolution, self.origin, self.trans)
            robot_map_path = os.path.join(self.data_dir, 'robot_map.png')  # Use .png extension
            cv2.imwrite(robot_map_path, robot_map)
            self.get_logger().info(f'Saved robot map image to {robot_map_path}')

            success = True

        except Exception as e:
            self.get_logger().error(f'Error during map generation: {e}')
            success = False

        ### Get New Coordinates ###

        if success:
            try:
                self.get_logger().info(f'Maps successfully generated, retrieving coordinates')

                # Encode the object_map image in-memory to Base64
                success_encode, encoded_image = cv2.imencode('.png', robot_map)
                if not success_encode:
                    raise ValueError("Failed to encode object map image to PNG format.")
                encoded_object_map = base64.b64encode(encoded_image).decode('utf-8')


                # 3 options: 
                # - oneCoordSearch
                # - divCoordSearch
                # - AstarCoordSearch
                COORDINATES_METHOD = "divCoordSearch"
                if COORDINATES_METHOD == "oneCoordSearch":

                    # Call the getcoord_search method with the request message and encoded image
                    self.get_logger().info(f'Get assistant reply')
                    assistant_reply = self.getcoord_search(request_msg, encoded_object_map)
                    
                    """assistant_reply = ""{
                        "success": "true",
                        "coordinates": {"x": 200, "y": 200},
                        "target_id": "table_001",
                        "error": "none",
                        "message": "The robot is oriented towards the center of the chair and ready to navigate."
                    }"""

                elif COORDINATES_METHOD == "divCoordSearch":

                    # Change prompt
                    self.INSTRUCTIONS = getcoord_prompt_id.getprompt()

                    # Call the getcoord_search method with the request message and encoded image
                    self.get_logger().info(f'Get assistant reply >> getId')
                    assistant_reply = self.getcoord_search(request_msg, encoded_object_map)

                    """assistant_reply = ""{
                        "success": "true",
                        "target_id": "table_001",
                        "error": "none",
                        "message": "The robot is oriented towards the center of the chair and ready to navigate."
                    }"""

                    # Get target_id
                    assistant_reply_json = json.loads(assistant_reply)
                    target_id = assistant_reply_json["target_id"]
                    self.get_logger().info(f'Assistant Reply >> getId: {assistant_reply}')

                    success = assistant_reply_json.get("success", "false")
                    if success == "true":
                        # load instructions, map and target name to get coordinates
                        self.INSTRUCTIONS_2 = getcoord_prompt_coord.getprompt()
                        messages = [
                            {"role": "system", "content": [{"type": "text", "text": f"{self.INSTRUCTIONS_2}"}]}
                        ]
                        messages.append({"role": "user", "content": [{"type": "text", "text": f"The map is: "}, {"type": "image_url","image_url": {"url":  f"data:image/jpeg;base64,{encoded_object_map}","detail": "high"},},],})
                        
                        item_pixelCoord_list = getcoord_pixelcoord_return.get(self.items_data, self.resolution, self.origin, object_map.shape[:2])
                        messages.append({"role": "assistant", "content": [{"type": "text", "text": f"json list of objects with height, width and coordinates of boxes (displayed on map), ensure the coordinate is not within any of the objects: {item_pixelCoord_list}"}]})
                        
                        messages.append({"role": "user", "content": [{"type": "text", "text": f"Find the new robot coordinates for the following item: {target_id}"}]})

                        messages.append({"role": "assistant", "content": [{"type": "text", "text": f"Please check all the sides of the object for a pixel that is part of the grid or white"}]})

                        # Call the getcoord_search method with the request message and encoded image
                        self.get_logger().info(f'Get assistant reply >> getCoord')
                        assistant_reply = ai_core.AI_Image_Prompt(messages, TEMPERATURE=1, MAX_TOKENS=300, FREQUENCY_PENALTY=0.7, PRESENCE_PENALTY=0.7)
                        assistant_reply = self.extract_json_block(assistant_reply)
                        self.get_logger().info(f'Assistant Reply >> Potential coordinates: {assistant_reply}')

                        ##################################

                        # Parse the JSON reply
                        data_json = json.loads(assistant_reply)
                        coordinates = data_json.get("coordinates", [])
                        
                        # Copy the map to draw on it
                        map_ax = object_map.copy()
                        
                        # Plot each coordinate on the map
                        for i, coord in enumerate(coordinates):
                            x, y = int(coord["x"]), int(coord["y"])
                            # Draw a circle for each coordinate
                            cv2.circle(map_ax, (x, y), radius=5, color=(0, 0, 255), thickness=-1)
                            # Add a label near each coordinate
                            label = f"Point {i+1}"
                            cv2.putText(map_ax, label, (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                        
                        # Save the updated map
                        map_ax_path = os.path.join(self.data_dir, 'map_ax.png')  # Use .png extension
                        cv2.imwrite(map_ax_path, map_ax)
                        
                        print(f'Saved object map image to {map_ax_path}')

                        #############################33

                        self.INSTRUCTIONS_3 = getcoord_prompt_check.getprompt()
                        messages = [
                            {"role": "system", "content": [{"type": "text", "text": f"{self.INSTRUCTIONS_3}"}]}
                        ]
                        messages.append({"role": "user", "content": [{"type": "text", "text": f"The map is: "}, {"type": "image_url","image_url": {"url":  f"data:image/jpeg;base64,{encoded_object_map}","detail": "high"},},],})
                        messages.append({"role": "user", "content": [{"type": "text", "text": f"{assistant_reply}"}]})

                        item_pixelCoord_list = getcoord_pixelcoord_return.get(self.items_data, self.resolution, self.origin, object_map.shape[:2])
                        messages.append({"role": "assistant", "content": [{"type": "text", "text": f"json list of objects with height, width and coordinates of boxes (displayed on map), ensure the coordinate is not within any of the objects: {item_pixelCoord_list}"}]})

                        # Find the most relevant point
                        assistant_reply = ai_core.AI_Image_Prompt(messages, TEMPERATURE=0.2, MAX_TOKENS=300, FREQUENCY_PENALTY=0, PRESENCE_PENALTY=0)
                        self.get_logger().info(f'Assistant Reply >> getCoord: {assistant_reply}')



                elif COORDINATES_METHOD == "AstarCoordSearch":
                    
                    # Change prompt
                    self.INSTRUCTIONS = getcoord_prompt_id.getprompt()

                    # Call the getcoord_search method with the request message and encoded image
                    self.get_logger().info(f'Get assistant reply')
                    assistant_reply = self.getcoord_search(request_msg, encoded_object_map)

                    """assistant_reply = ""{
                        "success": "true",
                        "target_id": "table_001",
                        "error": "none",
                        "message": "The robot is oriented towards the center of the chair and ready to navigate."
                    }"""

                    robot_coords = {"x": self.trans.transform.translation.x, "y": self.trans.transform.translation.y}

                    assistant_reply = getcoord_pathfind_return.get(object_map, self.resolution, self.origin, self.items_data, robot_coords, assistant_reply)

                    self.get_logger().info(f'robot coords: {robot_coords}\nassistant reply: {assistant_reply}')

                success = assistant_reply_json.get("success", "false")
                if success == "true":
                    # Generate new robot coordinates on the map for debugging
                    newCoords_map, self.angle_deg = getcoord_newcoordmap_generation.get(object_map, self.resolution, self.origin, assistant_reply, self.data)
                    newCoords_map_path = os.path.join(self.data_dir, 'newCoords_map.png')
                    cv2.imwrite(newCoords_map_path, newCoords_map)
                    self.get_logger().info(f'Saved new coordinates map image to {newCoords_map_path}')

                    # Get origin coordinates
                    response_message = getcoord_origincoord_return.get(assistant_reply, self.resolution, self.origin, object_map.shape[:2],self.angle_deg)
                    
                    # If success send info to chat
                    response_message_json = json.loads(response_message)
                    if response_message_json.get("success") == "true":
                        msg = String()
                        msg.data = response_message_json.get("message")
                        self.prompt_info_pub.publish(msg)

                    response.response = response_message
                    self.get_logger().info(f'Sending response: {response.response}')
                    return response
                else:
                    response.response = json.dumps(assistant_reply_json)
                    self.get_logger().info(f'11111111Sending failure response: {response.response}')
                    return response

            except Exception as e:
                self.get_logger().error(f'Error during coordinate processing: {e}')
                # Prepare a failure response with error details
                failure_response = {
                    "success": "false",
                    "error": str(e),
                    "message": f"Failed to process coordinates.{e}"
                }
                response.response = json.dumps(failure_response)
                return response
        else:
            # Prepare a failure response if map generation failed
            self.get_logger().error(f'Error during coordinate processing')
            failure_response = {
                "success": "false",
                "error": "Map generation failed.",
                "message": "Unable to generate the map due to an error."
            }
            response.response = json.dumps(failure_response)
            return response

    def extract_json_block(self, message: str) -> str:
        start = message.find('{')
        end = message.rfind('}')
        if start != -1 and end != -1:
            return message[start:end + 1]
        return ""


    def json_map_data_generation(self):

        # Parse the JSON string into a Python dictionary
        data = self.data
        
        # Initialize a new dictionary to hold the transformed data
        transformed_data = {
            "classes": data.get("classes", []),
            "items": {}
        }
        
        # Iterate over each class in the items
        for class_name, items in data.get("items", {}).items():
            # Extract only 'id' and 'description' for each item
            transformed_items = [
                {"id": item["id"], "description": item["description"]}
                for item in items
            ]
            # Assign the transformed list to the corresponding class
            transformed_data["items"][class_name] = transformed_items
        
        # Convert the transformed dictionary back to a JSON string
        self.map_data = json.dumps(transformed_data, indent=2)

        return

    # Method using LLMs to search for object
    def LLM_Search(self, object_class, object_description, error_log, object_map):

        # Initialize messages with system instructions
        messages = [
            {"role": "system", "content": [{"type": "text", "text": f"{self.INSTRUCTIONS}"}]}
        ]

        # Add the object list
        messages.append({"role": "user", "content": [{"type": "text", "text": f"The list of objects registered are: {self.map_data}"}]})

        # Add the map
        messages.append({"role": "user", "content": [{"type": "text", "text": f"The map is: "}, {"type": "image_url","image_url": {"url":  f"data:image/jpeg;base64,{object_map}"},},],})

        # Add the object and description
        messages.append({"role": "user", "content": [{"type": "text", "text": f"Return the coordinates for: {object_class}, with attributes: {object_description}"}]})
        
        if error_log != "":
            error_info = """
            An error has previously come up, below is the thread of messages between you and the user.
            Please use this information and try to determine the correct object and return its coordinates.
            If the object is still not clear, continue until success or until user asks to skip.
            """
            messages.append({"role": "system", "content": [{"type": "text", "text": f"{error_info}"}]})
            messages.append({"role": "assistant", "content": [{"type": "text", "text": f"{error_log}"}]})

        # Get response from ChatGPT
        
        try:
            assistant_reply = ai_core.AI_Image_Prompt(messages, TEMPERATURE=1, MAX_TOKENS=300, FREQUENCY_PENALTY=0, PRESENCE_PENALTY=0)

        except Exception as e:
            self.get_logger().info(f'Error sending data to LLM: {e}')
            assistant_reply = json.dumps({
                    "success": "false",
                    "error": "skip",
                    "message": f"Error sending data to LLM: {e}"
                })

        self.get_logger().info(f'assistant_reply: {assistant_reply}')
        return assistant_reply

    # Prompt Ai for coordinate
    def getcoord_search(self, message, object_map):
        # Get message attributes
        object_class = message.get("class")
        object_description = message.get("description", " ")
        error_log = ""
        
        # Check if the object class exists in the data
        classes = self.data.get('classes', [])
        if object_class in classes:
            items = self.data.get('items', {}).get(object_class, [])
            if len(items) == -1:
                item = items[0]
                assistant_reply = json.dumps({
                    "success": "true",
                    "coordinates": item['coordinates'],
                    "error": "none"
                })
                self.get_logger().info(f'Sending response: {assistant_reply}')
                return assistant_reply
            else:
                # Call LLM_Search and find coordinates
                self.get_logger().info(f'Requesting coord from AI for: {object_class}, with attributes: {object_description}')
                assistant_reply = self.LLM_Search(object_class, object_description, error_log, object_map)
                assistant_reply_json = json.loads(assistant_reply)

                # Get reply attributes 
                success = assistant_reply_json.get("success")
                error = assistant_reply_json.get("error")
                error_message = assistant_reply_json.get("message")

                trial = 0
                max_trials = 3
                handle_ambiguities_in_node = False

                while success == "false" and error != "skip" and trial < max_trials and handle_ambiguities_in_node:
                    self.get_logger().info(f'coord failure: {error_message}, prompting user')
                    # Reset the response
                    self.response = None
                    self.response_event.clear()

                    # Publish the request message to /prompt_request
                    msg = String()
                    msg.data = error_message
                    self.prompt_request_pub.publish(msg)
                    self.get_logger().info('Published to /prompt_request')

                    # Wait for the response with a timeout
                    if self.response_event.wait(timeout=60.0):
                        user_response = self.response
                        self.get_logger().info(f'User response: {user_response}')
                    else:
                        assistant_reply = json.dumps({
                            "success": "false",
                            "coordinates": {},
                            "error": "skip",
                            "message": "Operation timeout while waiting for reply."
                        })
                        self.get_logger().warn('No response received within timeout period.')
                        return assistant_reply

                    # Update the error log with the user's reply
                    error_log_add = (
                        f"\ntimestamp: {time.time()}\nError: {error_message}\n"
                        f"User response: {user_response}\n---"
                    )
                    error_log = f"{error_log}\n{error_log_add}"

                    # Call LLM_Search and find coordinates
                    assistant_reply = self.LLM_Search(object_class, object_description, error_log, object_map)
                    assistant_reply_json = json.loads(assistant_reply)

                    # Get reply attributes 
                    success = assistant_reply_json.get("success")
                    error = assistant_reply_json.get("error")
                    error_message = assistant_reply_json.get("message")

                    # Increase trial count
                    trial += 1

                self.get_logger().debug(f'Sending response: {assistant_reply}')
                return assistant_reply

        # Object class not found
        self.get_logger().info('Object class not found')
        assistant_reply = json.dumps({
            "success": "false",
            "error": "notExisting",
            "message": "The specified object class was not found."
        })
        return assistant_reply

    # Define the callback for the prompt_request
    def prompt_request_callback(self, future, mem_req):
        self.get_logger().info('prompt_request_callback invoked')

        # Process response and create a new search with updated information
        cli_response = future.result()
        error_reply = cli_response.response

        req_info = json.loads(mem_req) 
        object_class = req_info.get("object_class")
        object_description = req_info.get("object_description")
        error_log = req_info.get("error_log")

        # Update the error log with the user's reply
        error_log_new = f"""
        There was a previous error: {error}, the user was prompted: {error_message}, their response is {error_reply}
        Using this information, try to determine the correct object and return its coordinates.
        """
        error_log = f"{error_log_new}\n{error_log}"

        # Perform another LLM_Search with updated error log
        assistant_reply = self.LLM_Search(object_class, object_description, error_log)
        assistant_reply_json = json.loads(assistant_reply)

    # Get prompt response callback
    def prompt_response_callback(self, msg):
        self.get_logger().info(f'Received from /prompt_response: {msg.data}')
        if not self.response_event.is_set():
            self.response = msg.data
            self.response_event.set()



def main(args=None):
    rclpy.init(args=args)
    node = GetCoordNode()

    # Use a MultiThreadedExecutor with 2 threads
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    rclpy.spin(node, executor=executor)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
