# getcoord_robotmap_generation.py

import cv2
import numpy as np
import yaml
import random
import math

def get(cost_map, resolution, origin, trans):
    """
    Adds the robot's position and orientation to the cost map.

    Instead of drawing a red arrow, this function draws:
    - A red circle representing the robot's coordinates.
    - A green line indicating the robot's orientation.

    Parameters:
        cost_map (numpy.ndarray): The input cost map image, grayscale or color.

    Returns:
        numpy.ndarray: The cost map with the robot's position and orientation annotated.
    """
    # Check if the image is grayscale or color
    map_shape = cost_map.shape
    if len(map_shape) == 2:
        # Grayscale image
        map_height, map_width = map_shape
        robot_map_color = cv2.cvtColor(cost_map, cv2.COLOR_GRAY2BGR)
    elif len(map_shape) == 3:
        # Color image
        map_height, map_width, _ = map_shape
        robot_map_color = cost_map.copy()
    else:
        raise ValueError("Unexpected image shape for cost_map")

    # Extract position and orientation
    robot_x = trans.transform.translation.x
    robot_y = trans.transform.translation.y
    # Orientation as quaternion
    q = trans.transform.rotation

    # Manually compute yaw angle from quaternion
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    # Convert robot's position to map pixel coordinates
    origin_x, origin_y = origin[0], origin[1]
    map_x = int((robot_x - origin_x) / resolution)
    map_y = int((robot_y - origin_y) / resolution)
    map_y = map_height - map_y  # Adjust for image coordinate system

    # Parameters for drawing
    circle_radius = 5  # Radius of the red circle in pixels
    circle_color = (0, 0, 255)  # Red color in BGR
    circle_thickness = -1  # Filled circle

    line_length = 20  # Length of the orientation line in pixels
    line_color = (0, 255, 0)  # Green color in BGR
    line_thickness = 2  # Thickness of the orientation line

    # Define border
    border_thickness = 1
    border_color = (255, 0, 0)

    # Draw the red circle representing the robot's position
    cv2.circle(robot_map_color, (map_x, map_y), circle_radius + border_thickness, border_color, circle_thickness)
    cv2.circle(robot_map_color, (map_x, map_y), circle_radius, circle_color, circle_thickness)

    # Compute the end point of the orientation line
    end_x = int(map_x + line_length * math.cos(robot_yaw))
    end_y = int(map_y - line_length * math.sin(robot_yaw))  # Negative because y-axis is downward in image coordinates

    # Draw the green line indicating the robot's orientation
    cv2.line(robot_map_color, (map_x, map_y), (end_x, end_y), color=border_color, thickness=line_thickness + border_thickness)
    cv2.line(robot_map_color, (map_x, map_y), (end_x, end_y), color=line_color, thickness=line_thickness)

    return robot_map_color
