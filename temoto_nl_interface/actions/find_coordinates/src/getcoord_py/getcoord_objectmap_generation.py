# getcoord_objectmap_generation.py

import cv2
import numpy as np
import yaml
import random
import math

def get(robot_map, resolution, origin, items_data):
    
    # Map origin and resolution
    origin_x, origin_y = origin[0], origin[1]

    # Use the robot_map as the base image
    object_map_color = robot_map.copy()

    # Get map dimensions
    map_height, map_width, _ = object_map_color.shape

    # Create an overlay image for drawing the rectangles and lines
    overlay = object_map_color.copy()

    # Define padding for the rectangles (in pixels)
    rectangle_padding = 5  # Adjust as needed

    # Define border properties
    border_thickness = 1  # Thickness of the rectangle border in pixels
    border_color = (0, 0, 0)  # Black border

    # List to store label information to draw after all rectangles
    labels_to_draw = []

    # Iterate over each class and their items
    for item_class, items_list in items_data['items'].items():
        for item in items_list:
            item_id = item['id']
            coordinates = item['coordinates']
            dimensions = item['dimensions']

            # Convert world coordinates to map pixel coordinates
            map_x = int((coordinates['x'] - origin_x) / resolution)
            map_y = int((coordinates['y'] - origin_y) / resolution)

            # Adjust for image coordinate system (origin at top-left)
            map_y = map_height - map_y

            # Convert dimensions from meters to pixels
            width_px = int(dimensions['width'] / resolution)
            height_px = int(dimensions['height'] / resolution)

            # Define rectangle coordinates with padding
            top_left = (
                map_x - width_px // 2 - rectangle_padding,
                map_y - height_px // 2 - rectangle_padding
            )
            bottom_right = (
                map_x + width_px // 2 + rectangle_padding,
                map_y + height_px // 2 + rectangle_padding
            )

            # Ensure coordinates are within map bounds
            top_left = (
                max(0, min(map_width - 1, top_left[0])),
                max(0, min(map_height - 1, top_left[1]))
            )
            bottom_right = (
                max(0, min(map_width - 1, bottom_right[0])),
                max(0, min(map_height - 1, bottom_right[1]))
            )

            # Generate a random color for each item
            color = (
                random.randint(0, 255),  # Blue channel
                random.randint(0, 255),  # Green channel
                random.randint(0, 255)   # Red channel
            )

            # Draw the filled rectangle on the overlay image with the random color
            cv2.rectangle(overlay, top_left, bottom_right, color=color, thickness=-1)

            # Draw the rectangle border (outline) on the overlay
            cv2.rectangle(
                overlay,
                top_left,
                bottom_right,
                color=border_color,
                thickness=border_thickness
            )

            # Draw "X" lines inside the rectangle
            cv2.line(
                overlay,
                top_left,
                bottom_right,
                color=border_color,
                thickness=1  # Thin line
            )
            cv2.line(
                overlay,
                (top_left[0], bottom_right[1]),
                (bottom_right[0], top_left[1]),
                color=border_color,
                thickness=1  # Thin line
            )

            # Collect label information for later drawing
            labels_to_draw.append({
                'text': item_id,
                'top_left': top_left,
                'bottom_right': bottom_right,
                'color': color
            })

    # Now, draw all labels to ensure they are on top
    for label in labels_to_draw:
        item_id = label['text']
        top_left = label['top_left']
        bottom_right = label['bottom_right']
        color = label['color']

        # Define padding for the text inside the rectangle
        text_padding = 1  # pixels

        # Choose font and scale
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.3  # Adjusted for better readability
        thickness_text = 0  # Fixed thickness for text

        # Calculate size of the text
        (text_width, text_height), baseline = cv2.getTextSize(item_id, font, font_scale, thickness_text)

        # Calculate center position of the rectangle
        rect_center_x = (top_left[0] + bottom_right[0]) // 2
        rect_center_y = (top_left[1] + bottom_right[1]) // 2

        # Calculate text position to center it
        text_x = rect_center_x - (text_width // 2)
        text_y = rect_center_y + (text_height // 2)

        text_position = (text_x, text_y)

        # Text color should contrast with rectangle color
        # Calculate brightness of the rectangle color
        brightness = (color[0] + color[1] + color[2]) / 3
        if brightness < 128*100:
            text_color = (0, 255, 0)  # White for dark rectangles
            outline_color = (0, 0, 0)     # Black outline for text
        else:
            text_color = (0, 0, 0)        # Black for light rectangles
            outline_color = (255, 255, 255)  # White outline for text

        # Draw text outline for better visibility
        cv2.putText(
            overlay,
            item_id,
            text_position,
            font,
            font_scale,
            outline_color,
            thickness=3,  # Thicker outline
            lineType=cv2.LINE_AA
        )

        # Draw the actual text
        cv2.putText(
            overlay,
            item_id,
            text_position,
            font,
            font_scale,
            text_color,
            thickness=1,
            lineType=cv2.LINE_AA
        )

    # Apply transparency to the overlay
    alpha = 1  # Transparency factor (0.0 - 1.0)
    cv2.addWeighted(overlay, alpha, object_map_color, 1 - alpha, 0, object_map_color)

    # Return the modified object map
    return object_map_color
