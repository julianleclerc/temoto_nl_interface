import cv2
import numpy as np

def get(map_img, resolution, inflation_radius_m):
    
    # Convert inflation radius from meters to pixels
    inflation_radius_px = int(np.ceil(inflation_radius_m / resolution))

    # Identify obstacles and free space
    obstacles = (map_img == 0)      # Obstacles are black in the map image
    free_space = (map_img == 254)   # Free space is represented by 254 in the map image

    # Create a mask for distance transform: obstacles as 0, free space as 1
    distance_mask = np.where(obstacles, 0, 1).astype(np.uint8)

    # Compute the distance transform
    dist_transform = cv2.distanceTransform(distance_mask, cv2.DIST_L2, 5)

    # Convert distances from pixels to meters
    dist_transform_m = dist_transform * resolution

    # Initialize the cost map to 255 (white represents free space)
    cost_map = np.ones_like(map_img, dtype=np.uint8) * 255

    # Define the inflation zone (excluding obstacles)
    inflation_zone = (dist_transform_m > 0) & (dist_transform_m <= inflation_radius_m)

    # Assign gray (70) to the inflation zone
    cost_map[inflation_zone] = 70

    # Assign black (0) to obstacle cells
    cost_map[obstacles] = 0

    return cost_map
