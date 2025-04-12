#getcoord_pixelcoord_return.py

import copy

def get(items_data, resolution, origin, object_map_shape):
    

    # Extract origin coordinates and map dimensions
    origin_x, origin_y = origin[0], origin[1]
    height, width = object_map_shape  # height and width of the map image

    # Map to store items with pixel coordinates
    items_pixel_data = {
        "classes": items_data["classes"],
        "items": {}
    }

    # Iterate over each class
    for item_class in items_data["classes"]:
        items_pixel_data["items"][item_class] = []

        # Iterate over each item in the class
        for item in items_data["items"][item_class]:
            # Get world coordinates
            world_x = item["coordinates"]["x"]
            world_y = item["coordinates"]["y"]

            # Convert to pixel coordinates
            pixel_x = int((world_x - origin_x) / resolution)
            # Invert y-axis using the image height
            pixel_y = height - int((world_y - origin_y) / resolution) - 1

            # Create a new item with pixel coordinates
            item_pixel = copy.deepcopy(item)
            item_pixel["coordinates"] = {
                "x": pixel_x,
                "y": pixel_y
            }
            
            height = item["dimensions"]["height"]
            width = item["dimensions"]["width"]

            #item_pixel_dim["dimensions"] = {
            #   "height": height / resolution,
            #    "width": width / resolution
            #}

            # Append to the list
            items_pixel_data["items"][item_class].append(item_pixel)
            #items_pixel_data["items"][item_class].append(item_pixel_dim)

    return items_pixel_data
