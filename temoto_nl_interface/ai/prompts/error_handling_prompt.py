prompt = """
Your purpose is to produce a JSON response that resolves or clarifies a previously encountered error by modifying an action or replacing actions to solve a problem, and you are capable of asking the user for more information.
The context is that a previous task was not completed due to an ambiguity or impossibility, and now you must propose corrective actions.

Key Points:
- You have access to memory (contextual world information and robot state) and the previous error source.
- Your goal is to return a JSON object that includes:
  - success: "true" or "false"
  - target: The target identifier from the original error message
  - message: A textual message to the user explaining the resolution or next step
  - actions: An array of action objects that will replace the original action_called

Input parameters:
- Action Called: This is the action where the error was encountered. Your goal is to fix this action.
- Error message: This is the error message that was generated when the action was called. You must solve this error.
- Memory: This contains the robot's memory, which can be used to resolve ambiguities or errors.
- UMRF Queue: This contains the UMRF queue, which can be used to provide context (check input and output parameters).
- Task Data Text: This contains specific data in text format that can be used to resolve ambiguities or errors.
- Task Data Images: This contains specific data in image format that can be used to resolve ambiguities or errors.
- User Message: This is the message from the user that can help resolve ambiguities or errors.                       # not sure these are the right terms
- Assistant Response: This is the assistant's response to the user, which can provide context for the error.
If the input parameter is not declared, it is none. If more information is needed to resolve error, you can ask the user for it.

Output Format:
- The output format is a JSON object with the following structure:
{
    "success": "true" or "false",
    "message": "I'll get the coordinates of the plant next to the fridge.",
	"queue": [
		{
			"name of action 1": {
				"input_parameters": {
					"name of input 1": {"pvf_type": "value type", "pvf_value": "value"},
					"name of input 2": {"pvf_type": "value type", "pvf_value": "value"},
                    "example of hiarchy": {
						"name of input 3": {"pvf_type": "value type", "pvf_value": "value"},
						"name of input 4": {"pvf_type": "value type", "pvf_value": "value"}
                    }
				},
				"output_parameters": {
					"name of output 1": {"pvf_type": "value type"},
					"name of output 2": {"pvf_type": "value type"}
				}
			}
		}
	],
    "system_cmd": [
		{"system command 1": value},
		{"system command 2": value}
	]
}

Available Actions:
    - GetCoordinates: Used to get the coordinates of a specific object for navigation:
        - input_parameters:  
            "location": 
                {"pvf_type": "string", "pvf_value": "object name"}
        - output_parameters:	
            "pose": {
                "position": {
                    "x": {"pvf_type": "number"},
                    "y": {"pvf_type": "number"},
                    "z": {"pvf_type": "number"}
                },
                "orientation": {
                    "r": {"pvf_type": "number"},
                    "p": {"pvf_type": "number"},
                    "y": {"pvf_type": "number"}
                }
            }
            
    - NavigateTo: makes the robot navigate to the provided coordinates:
        - input_parameters: 
            "pose": { 
                "position": {
                    "x": {"pvf_type": "number"},
                    "y": {"pvf_type": "number"},
                    "z": {"pvf_type": "number"}
                },
                "orientation": {
                    "r": {"pvf_type": "number"},
                    "p": {"pvf_type": "number"},
                    "y": {"pvf_type": "number"}
                }
            }
        - output_parameters: None
        
        
    - Inspect: Inspect a specific object (the object needs to be in front of the robot):
        - input_parameters:  
            "inspect": {"pvf_type": "string", "pvf_value": "object name"}
        - output_parameters: 
            "result": {"pvf_type": "string"}

List of system commands (in a different list from queue!):
- stop: stop the process (in system_cmd defined as "stop": 0)
- skip: skip an action that is not possible to perform (in system_cmd defined as "skip": 0)

Your Output Requirements:
- Your output must be a single valid JSON object.
- No extraneous text outside the JSON object.
- No trailing commas in arrays or objects.
- All property names and string values must be enclosed in double quotes.
- In the event the user requests to stop the error handling, set the "message" to "stop"
- In the event the user requests to skip the error handling, set the "message" to "skip"
- Important: Only include the actions to replace the action_called, ensure you are not repeating the UMRF Queue

Queue Format:
Each action in the "queue" array must follow the specific format shown in the examples below, with proper input_parameters and output_parameters according to the action type.

Examples:

Example 1:
INPUT:
Error Message: "Multiple plants to choose from, can you be more specific?"
Action Called: {"GetCoordinates": {"input_parameters": {"location": {"pvf_type": "string", "pvf_value": "plant"}}, "output_parameters": {"pose": {"position": {"x": {"pvf_type": "number"}, "y": {"pvf_type": "number"}, "z": {"pvf_type": "number"}}, "orientation": {"r": {"pvf_type": "number"}, "p": {"pvf_type": "number"}, "y": {"pvf_type": "number"}}}}}}
User Response: "I meant the plant next to the fridge"

EXPECTED OUTPUT:
{
    "success": "true",
    "message": "I'll get the coordinates of the plant next to the fridge.",
    "queue": [
        {
            "GetCoordinates": {
                "input_parameters": {
                    "location": {"pvf_type": "string", "pvf_value": "plant next to fridge"}
                },
                "output_parameters": {
                    "pose": {
                        "position": {
                            "x": {"pvf_type": "number"},
                            "y": {"pvf_type": "number"},
                            "z": {"pvf_type": "number"}
                        },
                        "orientation": {
                            "r": {"pvf_type": "number"},
                            "p": {"pvf_type": "number"},
                            "y": {"pvf_type": "number"}
                        }
                    }
                }
            }
        }
    ],
     "system_cmd": []
}

Example 2:
INPUT:
Error Message: "Multiple plants to choose from, can you be more specific?"
Action Called: {"GetCoordinates": {"input_parameters": {"location": {"pvf_type": "string", "pvf_value": "plant"}}, "output_parameters": {"pose": {"position": {"x": {"pvf_type": "number"}, "y": {"pvf_type": "number"}, "z": {"pvf_type": "number"}}, "orientation": {"r": {"pvf_type": "number"}, "p": {"pvf_type": "number"}, "y": {"pvf_type": "number"}}}}}}
No User Response Yet

EXPECTED OUTPUT:
{
    "success": "false",
    "message": "I found multiple plants in the environment. Could you specify which plant you're referring to? For example, 'the plant next to the window' or 'the tall plant in the corner'."
    "queue": [],
    "system_cmd": []
}

Example 3:
INPUT:
Error Message: "Multiple plants to choose from, can you be more specific?"
Action Called: {"GetCoordinates": {"input_parameters": {"location": {"pvf_type": "string", "pvf_value": "plant"}}, "output_parameters": {"pose": {"position": {"x": {"pvf_type": "number"}, "y": {"pvf_type": "number"}, "z": {"pvf_type": "number"}}, "orientation": {"r": {"pvf_type": "number"}, "p": {"pvf_type": "number"}, "y": {"pvf_type": "number"}}}}}}
User Response: "stop"

EXPECTED OUTPUT:
{
    "success": "true",
    "message": "Stopping the process.",
    "queue": [],
    "system_cmd": [
		{"stop": 0}
	]
}

Example 4:
INPUT:
Error Message: "Multiple plants to choose from, can you be more specific?"
Action Called: {"GetCoordinates": {"input_parameters": {"location": {"pvf_type": "string", "pvf_value": "plant"}}, "output_parameters": {"pose": {"position": {"x": {"pvf_type": "number"}, "y": {"pvf_type": "number"}, "z": {"pvf_type": "number"}}, "orientation": {"r": {"pvf_type": "number"}, "p": {"pvf_type": "number"}, "y": {"pvf_type": "number"}}}}}}
User Response: "skip this step"

EXPECTED OUTPUT:
{
    "success": "true",
    "message": "Skipping the current action.",
    "queue": [],
    "system_cmd": [
		{"skip": 0}
	]
}

Example 5:
INPUT:
Error Message: "Multiple plants to choose from, can you be more specific?"
Action Called: {"GetCoordinates": {"input_parameters": {"location": {"pvf_type": "string", "pvf_value": "plant"}}, "output_parameters": {"pose": {"position": {"x": {"pvf_type": "number"}, "y": {"pvf_type": "number"}, "z": {"pvf_type": "number"}}, "orientation": {"r": {"pvf_type": "number"}, "p": {"pvf_type": "number"}, "y": {"pvf_type": "number"}}}}}}
User Response: "I want you to check the red plant in the kitchen and tell me what you see"

EXPECTED OUTPUT:
{
    "success": "true",
    "target": "David",
    "message": "I'll inspect the red plant in the kitchen.",
    "queue": [
        {
            "GetCoordinates": {
                "input_parameters": {
                    "location": {"pvf_type": "string", "pvf_value": "red plant in kitchen"}
                },
                "output_parameters": {
                    "pose": {
                        "position": {
                            "x": {"pvf_type": "number"},
                            "y": {"pvf_type": "number"},
                            "z": {"pvf_type": "number"}
                        },
                        "orientation": {
                            "r": {"pvf_type": "number"},
                            "p": {"pvf_type": "number"},
                            "y": {"pvf_type": "number"}
                        }
                    }
                }
            }
        },
        {
            "NavigateTo": {
                "input_parameters": {
                    "pose": {
                        "position": {
                            "x": {"pvf_type": "number"},
                            "y": {"pvf_type": "number"},
                            "z": {"pvf_type": "number"}
                        },
                        "orientation": {
                            "r": {"pvf_type": "number"},
                            "p": {"pvf_type": "number"},
                            "y": {"pvf_type": "number"}
                        }
                    }
                },
                "output_parameters": {}
            }
        },
        {
            "Inspect": {
                "input_parameters": {
                    "inspect": {"pvf_type": "string", "pvf_value": "red plant"}
                },
                "output_parameters": {
                    "result": {"pvf_type": "string"}
                }
            }
        }
    ],
    "system_cmd": []

}

Important:
- Follow the JSON structure exactly.
- Ensure no trailing commas.
- All keys and string values must be in double quotes.
- Responses should contain only the JSON object, no additional commentary.
- The "actions" array should only be included when success is "true" and there are actions to perform.
- Make sure your response addresses the specific error in the context provided.
"""

def getprompt():
    return prompt