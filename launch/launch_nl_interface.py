#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    # Launch arguments
    use_error_handler = LaunchConfiguration('use_error_handler', default='true')
    use_memory = LaunchConfiguration('use_memory', default='true')
    use_chat_interface = LaunchConfiguration('use_chat_interface', default='true')
    use_umrf_planner = LaunchConfiguration('use_umrf_planner', default='true')
    use_display_interface = LaunchConfiguration('use_display_interface', default='true')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_error_handler',
            default_value='true',
            description='Flag to enable/disable the error handler node'),
        DeclareLaunchArgument(
            'use_memory',
            default_value='true',
            description='Flag to enable/disable the memory node'),
        DeclareLaunchArgument(
            'use_chat_interface',
            default_value='true',
            description='Flag to enable/disable the chat interface node'),
        DeclareLaunchArgument(
            'use_umrf_planner',
            default_value='true',
            description='Flag to enable/disable the UMRF planner node'),
        DeclareLaunchArgument(
            'use_display_interface',
            default_value='true',
            description='Flag to enable/disable the display interface node'),
        
        # Start memory node first
        Node(
            package='temoto_nl_interface',
            executable='memory',
            name='memory_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(use_memory),
        ),
        
        # Wait a short time to ensure memory is initialized before starting other nodes
        TimerAction(
            period=2.0,
            actions=[
                # Start chat interface node
                Node(
                    package='temoto_nl_interface',
                    executable='chat_interface',
                    name='chat_interface_node',
                    output='screen',
                    emulate_tty=True,
                    condition=IfCondition(use_chat_interface),
                ),
            
                # Start UMRF planner node
                Node(
                    package='temoto_nl_interface',
                    executable='umrf_planner',
                    name='umrf_planner_node',
                    output='screen',
                    emulate_tty=True,
                    condition=IfCondition(use_umrf_planner),
                ),
                
                # Start display interface node
                Node(
                    package='temoto_nl_interface',
                    executable='display_interface',
                    name='display_interface_node',
                    output='screen',
                    emulate_tty=True,
                    condition=IfCondition(use_display_interface),
                ),

                # Start the error handler node
                Node(
                    package='temoto_nl_interface',
                    executable='error_handler',
                    name='error_handler_node',
                    output='screen',
                    emulate_tty=True,
                    condition=IfCondition(use_error_handler),
                ),
            ]
        ),
    ])