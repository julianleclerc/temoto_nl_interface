# temoto_nl_interface

ROS2 Natural Language Interface for TeMoto

## Installation Instructions

```bash
# Go to the source folder your ROS2 workspace
cd <PATH-TO-YOUR-ROS2-WS>/src

# Download this repository
git clone ...

# Build it
cd <PATH-TO-YOUR-ROS2-WS>
source /opt/ros/<YOUR-ROS2-DISTRO>/setup.bash
colcon build

# Set OpenAI key as environment variable:
export OPENAI_API_KEY=your_api_key_here
```

## Run the launch file

```bash
# Run the Natural Language Interface
ros2 launch temoto_nl_interface launch_nl_interface.py
```

## Overview

The `temoto_nl_interface` package provides a natural language interface for the TeMoto robotic system. It enables users to communicate with robots using natural language, parses these commands into actionable instructions, handles memory storage, manages error handling, and processes image feeds.

## Node Architecture

The package consists of the following main nodes:

1. **ChatInterface Node** - Natural language processing interface
2. **MemoryNode** - Memory management system
3. **FeedInterfaceNode** - Image data standardization and processing
4. **ErrorHandler** - Error handling and recovery
5. **UMRF_PLANNER** - Task planning and execution

## API Reference

### Topics

#### Chat Interface API

| Topic Name | Type | Description |
|------------|------|-------------|
| `/chat_interface_input` | `std_msgs/String` | Input channel for user messages. Format: JSON with "message", "targets", and "type" fields. |
| `/chat_interface_feedback` | `std_msgs/String` | Output channel for assistant responses. Format: JSON with "targets", "type", and "message" fields. |
| `/chat_action` | `std_msgs/String` | Output channel for action commands derived from chat. |

#### Memory API

| Topic Name | Type | Description |
|------------|------|-------------|
| `/add_to_memory` | `std_msgs/String` | Add information to the memory system. Format: JSON dictionary of memory entries. |
| `/remove_memory` | `std_msgs/String` | Remove information from memory. |

#### Feed Interface API

| Topic Name | Type | Description |
|------------|------|-------------|
| `/display_feed` | `std_msgs/String` | Input for image data. Format: JSON with "target", "name", and "image" fields. |
| `/webapp/display_feed/<target>/<name>` | `sensor_msgs/CompressedImage` | Output channel for processed image feeds. |

#### UMRF Planner API

| Topic Name | Type | Description |
|------------|------|-------------|
| `/chat_action` | `std_msgs/String` | Input channel for action commands. |
| `/umrf_correction` | `std_msgs/String` | Input channel for error corrections. |
| `/error_handling_message` | `std_msgs/String` | Output channel for error messages. |
| `/umrf_graph_feedback` | `temoto_msgs/UmrfGraphFeedback` | Feedback channel for task execution status. |
| `/umrf_graph_start` | `temoto_msgs/UmrfGraphStart` | Command channel to start task execution. |
| `/umrf_graph_stop` | `temoto_msgs/UmrfGraphStop` | Command channel to stop task execution. |
| `/umrf_graph_pause` | `temoto_msgs/UmrfGraphPause` | Command channel to pause task execution. |
| `/umrf_graph_resume` | `temoto_msgs/UmrfGraphResume` | Command channel to resume task execution. |
| `/umrf_graph_modify` | `temoto_msgs/UmrfGraphModify` | Command channel to modify ongoing tasks. |
| `/action_status` | `std_msgs/String` | Output channel for action status updates. |

#### Error Handling API

| Topic Name | Type | Description |
|------------|------|-------------|
| `/error_handling_message` | `std_msgs/String` | Input channel for error messages. |
| `/umrf_correction` | `std_msgs/String` | Output channel for error corrections. |

### Services

| Service Name | Type | Description |
|--------------|------|-------------|
| `/get_memory` | `nl_interface_msgs/Chat` | Service to retrieve relevant memory entries. |

### Message Formats

#### Chat Interface Input

```json
{
  "message": "User message text",
  "targets": ["target1", "target2"],
  "type": "request"
}
```

#### Chat Interface Output

```json
{
  "targets": ["target1", "target2"],
  "type": "response",
  "message": "Assistant response text"
}
```

#### Memory Storage Entry

```json
{
  "memory_name": {
    "type": "text",
    "data": "Memory content",
    "info": "Description of memory"
  }
}
```

#### UMRF Action Command

```json
{
  "targets": ["target1"],
  "success": "true",
  "message": "Action description",
  "queue": [
    {
      "action_name": {
        "input_parameters": {
          "param1": {
            "pvf_type": "string",
            "pvf_value": "value1"
          }
        }
      }
    }
  ],
  "system_cmd": []
}
```

#### Error Handling Message

```json
{
  "error_message": "Description of error",
  "action_called": "Action that failed",
  "runtime_messages": "Execution output",
  "umrf_queue": "Current queue state",
  "target": "target_name",
  "process_data_text": "Additional context",
  "process_data_images": "Image data"
}
```

## Configuration Options

The system has several configurable parameters:

- `MAX_CONTEXT`: Number of past questions considered in chat (default: 10)
- `MAX_CONCURRENT_REQUESTS`: Maximum parallel chat requests (default: 8)
- `jpeg_quality`: JPEG quality for compressed images (default: 85)
- `min_publish_interval`: Minimum time between image publications (default: 0.1 seconds)
- AI parameters (temperature, max_tokens, frequency_penalty, presence_penalty)

## Usage Examples

### Interacting with the Chat Interface

Send a message to a robot:

```bash
ros2 topic pub /chat_interface_input std_msgs/String "data: '{\"message\":\"Move to the kitchen\",\"targets\":[\"robot1\"],\"type\":\"request\"}'"
```

### Adding Memory

```bash
ros2 topic pub /add_to_memory std_msgs/String "data: '{\"kitchen_location\":{\"type\":\"text\",\"data\":\"The kitchen is located on the first floor\",\"info\":\"Information about kitchen location\"}}'"
```

### Displaying an Image Feed

```bash
ros2 topic pub /display_feed std_msgs/String "data: '{\"target\":\"robot1\",\"name\":\"camera\",\"image\":\"<base64-encoded-image>\"}'"
```

## Error Handling

The system provides robust error handling through the ErrorHandler node, which:

1. Processes errors encountered during task execution
2. Automatically attempts to solve errors when possible
3. Prompts the user for assistance when needed
4. Supports retrying, skipping, or modifying actions

## Customization

The package uses the OpenAI API for natural language processing. You can customize the behavior by modifying the prompt templates in the `temoto_nl_interface/ai/prompts` directory.

## Dependencies

- ROS2
- Python 3
- OpenAI API
- numpy
- opencv-python
- cv_bridge
- concurrent.futures