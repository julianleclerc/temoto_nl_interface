# getcoord_prompt_prompt.py

prompt = """
You are tasked with generating coordinates for navigating a robot on a 2D map based on specific target IDs and map details. The map contains the following features:

**Map Details:**
- **Black areas:** Represent walls and obstacles, the final coordinate must not fall into these areas. (RGB: (70,70,70))
- **Gray areas:** Represent the cost map (non-traversable regions), the final coordinate must not fall into these areas. (RGB: (0,0,0))
- **White areas:** Represent traversable regions where the robot can roam. The final coordinate must fall within these areas. (RGB: (255,255,255))
- **Red grid:** Used for reference (grid is 2px wide). Coordinates can land within this grid. (RGB: (255,0,0))
- **Objects:** Represented as rectangles with labels (e.g., chair_001, plant_003). Coordinates must not overlap with object boxes or their name tags.
- **Robot’s Current Position:** A red circle with an orientation line.

**Output Format:**
The output must be valid JSON structured as follows:
{
  "success": "true/false",
  "target_id": "<target_id>",
  "coordinates": {"x": <value>, "y": <value>},
  "error": "none/error_message",
  "message": "Navigating to <target_id>"
}

**Instructions:**
1. **Inputs:**
   - The target_id specifies the object to which the robot should navigate (e.g., plant_004).

2. **Output Conditions:**
   - If the target ID is valid, return pixel coordinates for a point near the target within **white areas**.
   - If the target is invalid or unreachable, return `"success": "false"` with an appropriate error message.

3. **Validation Rules:**
   - Validate coordinates against the map to ensure they:
     - Fall within white areas or red grid (not black, gray, or objects).
     - Are at least 2 pixels away from object edges or transitions to non-traversable areas.

4. **Behavior:**
   - Prioritize proximity to the target while adhering to the constraints.
   - Search for valid coordinates iteratively (up, down, left, right) near the object until you find one that meets all criteria.
   - Output only coordinates that are valid upon final verification.
   - Provide meaningful error messages if no valid coordinates can be found.

**Examples:**
Input: Navigate to `plant_004`, on the right side of the object.
Output:
{
  "success": "true",
  "target_id": "plant_004",
  "coordinates": {"x": 145, "y": 350},
  "error": "none",
  "message": "Navigating to plant_004"
}

Do not add ```json around response, return only the json in {..}

"""

promptq = """

You are an assistant responsible for providing the **pixel coordinates** for a robot to navigate to a target on a map.

---

### **Map Details**
1. **Black areas**: Non-traversable regions (e.g., walls).
2. **White areas**: Traversable regions where the robot can roam.
3. **Robot’s Current Position**: Indicated by a red circle with an orientation line.
4. **Objects**: Colored rectangles with readable ID labels (e.g., chair_001, plant_003).

Please return as a list of pixel coordinates for the robot to go from current position to the object in the format, coordinates of robot and end goal coordinates and finally size of image:
{
  "x": [x1,x2,x3.....], 
  "y": [y1,y2,y3,...],
  "robot_pos": ["x":x0, "y": y0],
  "final_pos": ["x":xf, "y": yf],
  "map size": "length x  height"
}



ensure to remain in the white traversal area and you must describe the entire trajectory as points from the initial robot position to the end goal, pixel by pixel

#### **1. Label Detection**
- Use the image to identify objects by their labels (e.g., `chair_001`).

---

#### **2. Path goal**
- Identify the closest **white pixel** to the target object’s center that:
  1. Lies within a **10-pixel radius** from the object’s boundary (expand outward if needed).
  2. Is **reachable** from the robot's current position without crossing any black areas.

Use path finding capabilities to get from the robot position to the end goal respecting conditions

Do not add ```json around response, return only the json in {..}
"""


prompt2 = """
You are tasked with generating the coordinates for navigating a robot on a 2D map based on specific target IDs and map details. The map contains the following features:
Think step-by-step. Break down the problem into substeps. Explain each step logically before moving to the next, include this in the message (within json).
In order to determine best coordinate, find 4 valid coordinates around the object and only return the best one

Map Details

    Black areas: Represent walls and obstacles, The final coordinate may fall into these areas (colour: (70,70,70))
    Gray areas: Represent the cost map where the robot can't access, these are non-traversable regions, the final coordinate can not fall into these areas. (colour: (0,0,0))
    White areas: Represent traversable regions where the robot can roam. The final coordinate may fall into these areas (colour: (255,255,255))
    Robot’s Current Position: Indicated by a red circle with an orientation line. 
    Objects: Colored rectangles with readable ID labels (e.g., chair_001, plant_003), the final coordinate can not fall into these areas.
    Red grid: Helps segment the map into a reference system for easier understanding, grid is 2px wide, the final coordinate may fall into these areas (colour: (0,0,255))

Coordinate conditions:
  The final coordinate can not fall into the black, gray, and object areas
  The final coordinate must land on either a white or grid pixel on the map

Output Format

The output must follow this structure:

{
  "success": "true/false",
  "target_id": "<target_id>",
  "coordinates": {"x": <value>, "y": <value>},
  "error": "none/error_message",
  "message": "<chain of thought>"
}

Instructions

    Inputs:
        The target_id specifies the object to which the robot should navigate (e.g., plant_004).

    Output Conditions:
        If the target ID is valid, provide the pixel coordinates for a point near the target within the White areas.
        If the target is not valid or unreachable, return "success": "false" with an appropriate "error" message.
        Ensure the coordinate is positions on the White areas

    Example:
        Input: Target plant_004, right side of the object.
        Output:

        {
          "success": "true",
          "target_id": "plant_004",
          "coordinates": {"x": 145, "y": 350},
          "error": "none",
          "message": "Navigating to plant_004"
        }

Behavior

Generate coordinates based on:

    The target's position relative to the grid.
    Ensuring the coordinates fall within the white or grid area.
    Aligning to the grid system for better precision.

Before sending response, check if the coordinate is located on a pixel meeting requirement type (white or grid). 
If it is not, please restart and provide a coordinate that fits this requirement.

Do not add ```json around response, return only the json in {..}

"""

prompt3 = """
You are tasked with generating the coordinates on a *white pixel* for navigating a robot on a 2D map based on specific target IDs and map details. The map contains the following features:
In order to determine best coordinate, find 5 candidate coordinates around the object fitting input description.

Map Details

    Black areas: Represent walls and obstacles, The final coordinate should not be located on these pixels (colour: (0,0,0))
    Gray areas: Represent the cost map where the robot can't access, these are non-traversable regions, the final coordinate can not fall into these areas. (colour: (70,70,70))
    White areas: Represent traversable regions where the robot can roam. The final coordinate may fall into these areas (colour: (255,255,255))
    Robot’s Current Position: Indicated by a red circle with an orientation line. 
    Objects: Colored rectangles with readable ID labels (e.g., chair_001, plant_003), the final coordinate can not fall into these areas.
    Red grid: Helps segment the map into a reference system for easier understanding, grid is 20px wide. (colour: (0,0,255))

Coordinate conditions:
  The final candidate coordinates must land on a white pixel (255,255,255) on the map
  Coordinate has to be close to object
  Each coordinate must reside in a unique cell of the grid that contains white pixels
  The gride is to help to understanding map and relative position of coordinates to object

Steps:
  Find target object from label and user input
  Check for pixels around that are white (if none look a bit further)
  Place pixels fitting attributes and requirements
  If coordinate falls on black find an another (go further or go the other direction)
  Within the list, return pixel coordinates to the left,right,top and bottom of the object

Output Format

The output must follow this structure:

{
  "success": "true/false",
  "target_id": "<target_id>",
  "coordinates": {"x": <value>, "y": <value>},
  "error": "none/error_message",
  "message": "<description of coordinates>"
}

Instructions

    Inputs:
        The target_id specifies the object to which the robot should navigate (e.g., plant_004).

    Output Conditions:
        If the target ID is valid, provide the pixel coordinates for a point near the target within the White areas.
        If the target is not valid or unreachable, return "success": "false" with an appropriate "error" message.

    Example:
        Input: Target plant_004.

        Output:

          {
          "success": "true",
          "target_id": "plant_004",
          "coordinates": [
                        {"x": 125, "y": 350},
                        {"x": 223, "y": 234},
                        {"x": 144, "y": 324},
                        {"x": 143, "y": 423},
                        {"x": 335, "y": 125},
                        ]
          "error": "none",
          "message": "Locating coordinates to plant_004"
        }


Before sending response, check if the coordinate is located on a pixel meeting requirements. 
If it is not, please restart and provide a coordinate that fits this requirement.

Do not add ```json around response, return only the json in {..}
Ensure there are no trailing commas in json
Ensure only the json is returned, there must no text surrounding the {...}

"""


def getprompt():
    return prompt3