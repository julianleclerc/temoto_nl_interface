import numpy as np
import cv2

def get(map_img, resolution, origin, robot_position):
    """
    Modifies the map image by setting all non-traversable areas (not reachable from the robot's position)
    to black (0, 0, 0), without altering the colors of traversable areas.

    Args:
        map_img (numpy.ndarray): The input map image (color image).
        resolution (float): The map's resolution in meters per pixel.
        origin (tuple): The (x, y) world coordinates of the map's origin.
        robot_position: The robot's position with world coordinates.

    Returns:
        numpy.ndarray: The modified map image with unreachable areas set to black.
    """
    # Extract robot's position in world coordinates
    robot_x = robot_position.transform.translation.x
    robot_y = robot_position.transform.translation.y

    # Map size in pixels
    height, width = map_img.shape[:2]

    # Convert robot's position to pixel coordinates
    origin_x, origin_y = origin[0], origin[1]
    pixel_x = int((robot_x - origin_x) / resolution)
    pixel_y = height - int((robot_y - origin_y) / resolution) - 1

    # Ensure pixel_x and pixel_y are within image bounds
    pixel_x = np.clip(pixel_x, 0, width - 1)
    pixel_y = np.clip(pixel_y, 0, height - 1)

    # Create free space mask: pixels with values 254 or 255 in all channels are free space
    free_space_mask = (
        np.all(map_img == [254, 254, 254], axis=2) |
        np.all(map_img == [255, 255, 255], axis=2)
    ).astype(np.uint8)

    # Prepare image for flood fill
    flood_fill_img = free_space_mask.copy()

    # Perform flood fill starting from the robot's pixel position
    # We fill the area with the value 2
    cv2.floodFill(flood_fill_img, None, (pixel_x, pixel_y), 2)

    # Create a mask of unreachable free spaces: areas not filled by flood fill
    unreachable_mask = (flood_fill_img != 2) & (free_space_mask == 1)

    # Set unreachable free space areas to black in the original map image
    map_img[unreachable_mask] = [0, 0, 0]

    return map_img
