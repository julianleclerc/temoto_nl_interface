#getcoord_scalemap_generation.py

import cv2
import numpy as np
import yaml
import random
import math

def get(map_img, resolution, scale_factor):

    # Get original dimensions
    original_height, original_width = map_img.shape[:2]

    # Calculate new dimensions
    new_width = max(1, int(original_width * scale_factor))
    new_height = max(1, int(original_height * scale_factor))

    # Choose interpolation method based on scale factor
    if scale_factor > 1:
        interpolation = cv2.INTER_NEAREST
    elif scale_factor < 1:
        interpolation = cv2.INTER_AREA
    else:
        # scale_factor == 1, return the original image
        return map_img.copy(), resolution

    # Perform the scaling
    scaled_img = cv2.resize(
        map_img,
        (new_width, new_height),
        interpolation=interpolation
    )

    # Update resolution
    new_resolution = resolution / scale_factor

    # Determine the size for the square image
    max_dim = max(new_width, new_height)

    # Initialize a black square image
    if len(scaled_img.shape) == 3:
        # For color images
        square_img = np.zeros((max_dim, max_dim, scaled_img.shape[2]), dtype=scaled_img.dtype)
    else:
        # For grayscale images
        square_img = np.zeros((max_dim, max_dim), dtype=scaled_img.dtype)

    # Place the scaled image onto the square canvas
    square_img[:new_height, :new_width] = scaled_img

    return square_img, new_resolution
