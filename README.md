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
SET OPENAI_API_KEY=your_api_key_here

```

# Run the launch file

```bash
# Run the Natural Language Interface
ros2 launch temoto_nl_interface launch_nl_interface.py
```

