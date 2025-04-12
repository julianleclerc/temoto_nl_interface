# getcoord_prompt_prompt.py

prompt3 = """
You are tasked with determining the best coordinate relating to the position of a robot needing to go to a particular object

Map Details

    Black areas: Represent walls and obstacles, The final coordinate should not be located on these pixels (colour: (0,0,0))
    Gray areas: Represent the cost map where the robot can't access, these are non-traversable regions, The final coordinate should not be located on these pixels. (colour: (70,70,70))
    White areas: Represent traversable regions where the robot can roam. The final coordinate may fall into these areas (colour: (255,255,255))
    Objects: Colored rectangles with readable ID labels (e.g., chair_001, plant_003), The final coordinate should not be located on these pixels.
    Red grid: Helps segment the map into a reference system for easier understanding, grid is 2px wide, the final coordinate may fall into these areas (colour: (0,0,255))
    Robot’s Current Position: Indicated by a red circle with an orientation line. 


Steps:
  Check relation between the coordinates and the target on the map
  Pick which coordinate is the most fitting message and context
  Return json to user

Conditions to follow:
  The final coordinate must be on a white (255,255,255) or red pixel (0,0,255) on the map (if not, discard it)
  The labels 001/002/003... does not indicate the coordinate index, the decision must be made looking at the map


Output Format

The output must follow this structure:

{
  "success": "true/false",
  "target_id": "<target_id>",
  "coordinates": {"x": <value>, "y": <value>},
  "error": "none/error_message",
  "message": "<chain of thought>"
}

Inputs:
    The target_id specifies the object to which the robot should navigate.
    The message specifies the relation the robot needs to be to the object.
    The coordinates is a list of 5 different potential coordinates that have been generated.

Output Conditions:
    If the target ID is valid, provide the pixel coordinates for a point near the target within the White areas.
    If the target is not valid or unreachable, return "success": "false" with an appropriate "error" message.
    Ensure the coordinate is positions on the White or grid areas areas

Example:
    Input:
      {
      "success": "true",
      "target_id": "plant_004",
      "coordinates": [
                    {"x": 145, "y": 350},
                    {"x": 123, "y": 234},
                    {"x": 144, "y": 324},
                    {"x": 143, "y": 323},
                    {"x": 155, "y": 365},
                    ]
      "error": "none",
      "message": "Found 5 coordinates to go to plant_004"
    }

    Output:
    {
      "success": "true",
      "target_id": "plant_004",
      "coordinates": {"x": 123, "y": 234},
      "error": "none",
      "message": "Going to plant_004"
    }

If none of the ouputs are correct, determine a coordinate that would meet the requirements, include in message that none of the given coordinates were good
Errors include:
  - too far from object
  - too close to object
  - not respecting the requested orientation relative to object
  - overlapping with other objects
  - On a black pixel, on a gray pixel

Do not add ```json around response, return only the json in {..}
"""


def getprompt():
    return prompt3