#getcoord_grid_generation.py

import numpy as np
import cv2

def get2(cost_map, resolution, grid_scale):
    
    # Calculate pixels per grid cell
    pixels_per_grid_cell = grid_scale  # Assuming grid_scale is in pixels

    # Map dimensions
    height, width = cost_map.shape[:2]

    # Create a copy of the map to draw the grid on
    grid_map = cost_map.copy()

    # Ensure cost_map is in grayscale
    if len(cost_map.shape) == 3 and cost_map.shape[2] == 3:
        cost_map_gray = cv2.cvtColor(cost_map, cv2.COLOR_BGR2GRAY)
    else:
        cost_map_gray = cost_map.copy()

    # Threshold to binary if necessary (assuming white is 255 and black is 0)
    _, binary_map = cv2.threshold(cost_map_gray, 254, 255, cv2.THRESH_BINARY)

    # Define grid line color
    grid_color = (0, 0, 255) 

    # Generate x positions for vertical grid lines
    x_positions = np.arange(0, width, pixels_per_grid_cell).astype(int)

    # Draw vertical grid lines only on white areas
    for x in x_positions:
        # Iterate over each y to set the pixel if it's white
        for y in range(height):
            #if binary_map[y, x] == 255:
            grid_map[y, x] = grid_color

    # Generate y positions for horizontal grid lines
    y_positions = np.arange(0, height, pixels_per_grid_cell).astype(int)

    # Draw horizontal grid lines only on white areas
    for y in y_positions:
        # Iterate over each x to set the pixel if it's white
        for x in range(width):
            #if binary_map[y, x] == 255:
            grid_map[y, x] = grid_color

    return grid_map





def get(cost_map, resolution, grid_scale):
    
    # Calculate pixels per grid cell
    pixels_per_grid_cell = grid_scale  # Assuming grid_scale is in pixels

    # Map dimensions
    height, width = cost_map.shape[:2]

    # Create a copy of the map to draw the grid on
    grid_map = cost_map.copy()

    # Ensure cost_map is in grayscale
    if len(cost_map.shape) == 3 and cost_map.shape[2] == 3:
        cost_map_gray = cv2.cvtColor(cost_map, cv2.COLOR_BGR2GRAY)
    else:
        cost_map_gray = cost_map.copy()

    # Threshold to binary if necessary (assuming white is 255 and black is 0)
    _, binary_map = cv2.threshold(cost_map_gray, 254, 255, cv2.THRESH_BINARY)

    # Define grid line color
    grid_color = (0, 0, 255)  # Gray color

    # Generate x positions for vertical grid lines
    x_positions = np.arange(0, width, pixels_per_grid_cell).astype(int)

    # Draw vertical grid lines only on white areas
    for x in x_positions:
        # Iterate over each y to set the pixel if it's white
        for y in range(height):
            if binary_map[y, x] == 255:
                grid_map[y, x] = grid_color

    # Generate y positions for horizontal grid lines
    y_positions = np.arange(0, height, pixels_per_grid_cell).astype(int)

    # Draw horizontal grid lines only on white areas
    for y in y_positions:
        # Iterate over each x to set the pixel if it's white
        for x in range(width):
            if binary_map[y, x] == 255:
                grid_map[y, x] = grid_color

    return grid_map
