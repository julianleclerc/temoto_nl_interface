#getcoord_newcoordmap_generation.py

import cv2
import numpy as np
import yaml
import random
import math
import json

def get(object_map, resolution, origin, assistant_reply, data):
    """
    Places the robot's new coordinates on the map and computes the orientation to the target.

    The robot's position is marked with a green circle, and an arrow is drawn pointing towards the target.

    Parameters:
        object_map (numpy.ndarray): The base map image (BGR format) where the annotations will be added.
        assistant_reply (str): A JSON-formatted string containing the new coordinates and target information.

    Returns:
        numpy.ndarray: The updated map image with the robot's new position and orientation annotated.
    """
    # Parse assistant_reply
    try:
        assistant_reply = json.loads(assistant_reply)
    except json.JSONDecodeError as e:
        return object_map, 0

    success = assistant_reply.get("success", "false").lower()

    if success != "true":
        error_message = assistant_reply.get("message", "Failed to get new coordinates -> skip new coord map.")
        return object_map, 0

    coordinates = assistant_reply.get("coordinates")
    target_id = assistant_reply.get("target_id")
    if not coordinates:
        return object_map, 0

    no_target = not bool(target_id)

    try:
        x_robot = int(coordinates["x"])
        y_robot = int(coordinates["y"])
    except (KeyError, TypeError, ValueError) as e:
        return object_map, 0

    # Retrieve target coordinates from data
    if not no_target:
        target_coords = get_target_coordinates(target_id, data)
        if not target_coords:
            return object_map, 0

        x_target_world = target_coords["x"]
        y_target_world = target_coords["y"]

        # Convert target world coordinates to pixel coordinates
        origin_x, origin_y = origin[0], origin[1]
        resolution = resolution  # meters per pixel
        map_height, map_width = object_map.shape[:2]

        # Conversion from world to pixel coordinates
        x_target_px = int((x_target_world - origin_x) / resolution)
        y_target_px = int((y_target_world - origin_y) / resolution)

        # Adjust y coordinate if necessary (image coordinates y increase downwards)
        y_target_px = map_height - y_target_px

        # Calculate orientation (angle) from robot to target
        delta_x = x_target_px - x_robot
        delta_y = y_target_px - y_robot
        angle_rad = math.atan2(delta_y, delta_x)
        angle_deg = math.degrees(angle_rad)

    # Ensure that the coordinates are within the map boundaries
    map_height, map_width = object_map.shape[:2]
    if not (0 <= x_robot < map_width and 0 <= y_robot < map_height):
        return object_map, angle_deg

    if not no_target:
        if not (0 <= x_target_px < map_width and 0 <= y_target_px < map_height):
            return object_map, angle_deg

    # Parameters for drawing
    circle_radius = 5       # Radius of the green circle in pixels
    circle_color = (0, 255, 0)  # Green color in BGR
    circle_thickness = -1   # Filled circle
    arrow_color = (255, 0, 0)   # Blue color in BGR
    arrow_thickness = 2

    # Draw the green circle representing the robot's new position
    cv2.circle(object_map, (x_robot, y_robot), circle_radius, circle_color, circle_thickness)

    if not no_target:
        # Draw an arrow from the robot to the target
        cv2.arrowedLine(object_map, (x_robot, y_robot), (x_target_px, y_target_px), arrow_color, arrow_thickness)

    return object_map, angle_deg


def get_target_coordinates(target_id, data):
    """
    Retrieves the target's coordinates from self.data using the target_id.

    Parameters:
        target_id (str): The ID of the target object.

    Returns:
        dict: A dictionary with 'x' and 'y' keys representing the target's coordinates, or None if not found.
    """
    for item_type, items in data.get("items", {}).items():
        for item in items:
            if item.get("id") == target_id:
                coordinates = item.get("coordinates")
                if coordinates and "x" in coordinates and "y" in coordinates:
                    return {"x": coordinates["x"], "y": coordinates["y"]}
    return None
