# getcoord_origincoord_return.py

import cv2
import numpy as np
import yaml
import random
import math
import json


def get(assistant_reply, resolution, origin, map_shape, angle):
    """
    Converts coordinates from the image coordinate system (origin at top-left) to real-world coordinates.

    Parameters:
        assistant_reply (str): A JSON-formatted string containing coordinates in the image coordinate system.

    Returns:
        str: A JSON-formatted string with coordinates converted to the real-world coordinate system.
    """

    try:
        map_height, map_width = map_shape

        # Parse assistant_reply
        assistant_reply_dict = json.loads(assistant_reply)
        coords = assistant_reply_dict.get('coordinates', {})
        x_img = coords.get('x')
        y_img = coords.get('y')

        if x_img is None or y_img is None:
            # Handle error
            error_message = "Invalid coordinates in assistant reply."
            return json.dumps({
                "success": "false",
                "error": "InvalidCoordinates",
                "message": error_message
            })

        # Get origin and resolution
        origin_x, origin_y = origin[0], origin[1]
        resolution = resolution

        # Convert image coordinates to real-world coordinates
        x_world = x_img * resolution + origin_x
        y_world = (map_height - y_img) * resolution + origin_y  # Adjusted line

        # Update the assistant_reply_dict with world coordinates
        assistant_reply_dict['coordinates']['x'] = x_world
        assistant_reply_dict['coordinates']['y'] = y_world

        # Include angle to response
        assistant_reply_dict['angle'] = angle

        assistant_reply_updated = json.dumps(assistant_reply_dict)

        # Return the updated response as JSON
        return assistant_reply_updated  # Removed unnecessary json.dumps

    except Exception as e:
        error_message = f"Error converting coordinates: {e}"
        return json.dumps({
            "success": "false",
            "error": "ConversionError",
            "message": error_message
        })
