#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import json
import base64
import numpy as np
import time

class WebcamPublisher(Node):
    """
    ROS2 node that captures frames from a webcam and publishes them
    to the /display_feed topic in the format expected by FeedInterface.
    """
    def __init__(self):
        super().__init__('webcam_publisher')
        
        # Create publisher for display feed
        self.publisher = self.create_publisher(
            String,
            '/display_feed',
            10
        )
        
        # Create parameters
        self.declare_parameter('camera_id', 0)  # Default to first camera
        self.declare_parameter('frame_rate', 10.0)  # FPS
        self.declare_parameter('resolution_width', 640)
        self.declare_parameter('resolution_height', 480)
        self.declare_parameter('camera_name', 'webcam')
        
        # Get parameters
        self.camera_id = self.get_parameter('camera_id').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.width = self.get_parameter('resolution_width').value
        self.height = self.get_parameter('resolution_height').value
        self.camera_name = self.get_parameter('camera_name').value
        
        # Initialize webcam
        self.get_logger().info(f'Initializing webcam with ID: {self.camera_id}')
        self.cap = cv2.VideoCapture(self.camera_id)
        
        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        
        # Check if webcam opened successfully
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open webcam with ID: {self.camera_id}')
            raise RuntimeError(f"Could not open webcam with ID: {self.camera_id}")
        
        # Get actual resolution (may differ from requested)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Webcam resolution: {self.width}x{self.height}')
        
        # Create timer for frame capture
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Webcam publisher initialized')

    def timer_callback(self):
        """Capture a frame and publish it to the /display_feed topic."""
        try:
            # Capture frame
            ret, frame = self.cap.read()
            
            if not ret:
                self.get_logger().error('Failed to capture frame from webcam')
                return
            
            # Convert to RGB for consistency (OpenCV uses BGR by default)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Method 1: Publish as base64-encoded string
            # Convert to JPEG and then base64
            _, buffer = cv2.imencode('.jpg', frame_rgb)
            base64_data = base64.b64encode(buffer).decode('utf-8')
            
            # Create message
            msg_data = {
                'target': 'David',
                'name': self.camera_name,
                'image': base64_data
            }
            
            # Alternative method (uncomment to use):
            # Method 2: Publish as raw image data with encoding
            # msg_data = {
            #     'name': self.camera_name,
            #     'image': {
            #         'data': base64.b64encode(frame_rgb.tobytes()).decode('utf-8'),
            #         'height': self.height,
            #         'width': self.width,
            #         'encoding': 'rgb8'
            #     }
            # }
            
            # Convert to JSON and publish
            msg = String()
            msg.data = json.dumps(msg_data)
            self.publisher.publish(msg)
            
            self.get_logger().debug(f'Published frame with name: {self.camera_name}')
            
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')

    def __del__(self):
        # Release webcam when node is destroyed
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Webcam released')

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    
    try:
        rclpy.spin(webcam_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        webcam_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()