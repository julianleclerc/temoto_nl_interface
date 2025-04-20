#!/usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class FakeNavigationServer(Node):
    def __init__(self):
        super().__init__('fake_navigation_server')
        self.get_logger().info('Starting Fake Navigation Server')
        
        # Use a ReentrantCallbackGroup to allow for concurrent callbacks
        callback_group = ReentrantCallbackGroup()
        
        # Create the action server for NavigateToPose
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback,
            callback_group=callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info('Fake Navigation Server is ready')
    
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return rclpy.action.GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')
        
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()
        
        # Get the target pose from the goal
        goal = goal_handle.request
        target_pose = goal.pose
        
        self.get_logger().info(f'Target position: x={target_pose.pose.position.x}, '
                               f'y={target_pose.pose.position.y}, '
                               f'z={target_pose.pose.position.z}')
        
        # Set current pose for feedback (initially at starting position)
        current_pose = target_pose
        
        # Simulate navigation with a 5-second pause
        start_time = self.get_clock().now()
        time_to_wait = 5.0  # seconds
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < time_to_wait:
            # Check if goal is being canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return NavigateToPose.Result()
            
            # Update feedback with current "position"
            # In a real system, this would be the robot's current pose
            feedback_msg.current_pose = current_pose
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate some processing
            time.sleep(0.1)
        
        # Successfully completed the fake navigation
        goal_handle.succeed()
        self.get_logger().info('Goal succeeded!')
        
        # Return an empty result
        return result


def main(args=None):
    rclpy.init(args=args)
    
    fake_nav_server = FakeNavigationServer()
    
    # Use a multi-threaded executor to allow processing of callbacks in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(fake_nav_server)
    
    try:
        fake_nav_server.get_logger().info('Fake Navigation Server is running. Ctrl-C to exit.')
        executor.spin()
    except KeyboardInterrupt:
        fake_nav_server.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        # Clean up
        executor.shutdown()
        fake_nav_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()