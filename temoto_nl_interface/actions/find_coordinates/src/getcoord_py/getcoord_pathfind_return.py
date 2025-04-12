# getcoord_pathfind_return.py

import json
import heapq
import numpy as np

def get(object_map, resolution, origin, items_data, robot_coords, assistant_reply):
    # Parse assistant_reply to get target_id
    assistant_data = json.loads(assistant_reply)
    target_id = assistant_data.get('target_id')

    # Find the target item coordinates in items_data
    item_coords = None
    for item_class, items_list in items_data.get('items', {}).items():
        for item in items_list:
            if item.get('id') == target_id:
                item_coords = item.get('coordinates')
                break
        if item_coords is not None:
            break

    if item_coords is None:
        assistant_data['success'] = "false"
        assistant_data['error'] = "Target ID not found in items_data"
        assistant_data['message'] = "The specified target could not be found."
        return json.dumps(assistant_data)

    # Get map dimensions
    map_height, map_width = object_map.shape[:2]  # Grayscale image has shape (height, width)

    # Extract origin coordinates (only x and y)
    origin_x, origin_y = origin[:2]  # Assuming origin is a list or tuple

    # Convert robot_coords and item_coords to map indices
    def world_to_map(x, y):
        col = int((x - origin_x) / resolution)
        row = map_height - int((y - origin_y) / resolution) - 1
        return (row, col)

    robot_x = robot_coords.get('x')
    robot_y = robot_coords.get('y')
    robot_position = world_to_map(robot_x, robot_y)

    item_x = item_coords.get('x')
    item_y = item_coords.get('y')
    item_position = world_to_map(item_x, item_y)

    # Check if indices are within map bounds
    def is_within_bounds(position):
        row, col = position
        return 0 <= row < map_height and 0 <= col < map_width

    if not is_within_bounds(robot_position):
        assistant_data['success'] = "false"
        assistant_data['error'] = "Robot coordinates out of map bounds"
        assistant_data['message'] = "The robot's position is outside the map boundaries."
        return json.dumps(assistant_data)

    if not is_within_bounds(item_position):
        assistant_data['success'] = "false"
        assistant_data['error'] = "Item coordinates out of map bounds"
        assistant_data['message'] = "The item's position is outside the map boundaries."
        return json.dumps(assistant_data)

    # Proceed with pathfinding
    class Node:
        def __init__(self, position, parent=None):
            self.position = position  # (row, col)
            self.parent = parent
            self.g = 0  # Cost from start to current node
            self.h = 0  # Estimated cost from current to goal
            self.f = 0  # Total cost

        def __lt__(self, other):
            return self.f < other.f

    def heuristic(a, b):
        # Use Euclidean distance as the heuristic
        return ((a[0] - b[0])**2 + (a[1] - b[1])**2) ** 0.5

    def is_walkable(position):
        row, col = position
        # Check if the position is within bounds and is a walkable (white) area
        if is_within_bounds(position):
            pixel = object_map[row][col]
            if object_map.ndim == 3:
                # For color images, check if all channels are 255
                return np.all(pixel == 255)
            else:
                # For grayscale images, check if the pixel is 255
                return pixel == 255
        return False

    # Initialize the open and closed lists
    open_list = []
    closed_set = set()

    start_node = Node(robot_position)
    start_node.h = heuristic(robot_position, item_position)
    start_node.f = start_node.h

    min_distance = start_node.h
    best_node = start_node

    heapq.heappush(open_list, start_node)

    # Define movement directions (up, down, left, right)
    movements = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while open_list:
        current_node = heapq.heappop(open_list)
        current_position = current_node.position

        if current_position in closed_set:
            continue

        closed_set.add(current_position)

        # Update the best_node if closer to the item
        current_distance = heuristic(current_position, item_position)
        if current_distance < min_distance:
            min_distance = current_distance
            best_node = current_node

        # If we reach the item position (unlikely since it's unreachable), we can break
        if current_position == item_position:
            best_node = current_node
            break

        # Explore neighbors
        for move in movements:
            neighbor_row = current_position[0] + move[0]
            neighbor_col = current_position[1] + move[1]
            neighbor_position = (neighbor_row, neighbor_col)

            if not is_walkable(neighbor_position) or neighbor_position in closed_set:
                continue

            neighbor_node = Node(neighbor_position, current_node)
            neighbor_node.g = current_node.g + 1
            neighbor_node.h = heuristic(neighbor_position, item_position)
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            heapq.heappush(open_list, neighbor_node)

    # **Return Map Coordinates Instead of World Coordinates**
    final_coords = {
        'x': best_node.position[1],  # col index as x-coordinate
        'y': best_node.position[0]   # row index as y-coordinate
    }

    # Update assistant_reply with the new map coordinates
    assistant_data['coordinates'] = {
        'x': final_coords['x'],
        'y': final_coords['y']
    }

    # Ensure success is true and error is none
    assistant_data['success'] = "true"
    assistant_data['error'] = "none"

    # Convert back to string
    updated_assistant_reply = json.dumps(assistant_data)

    return updated_assistant_reply
