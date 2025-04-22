#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class WebcamPublisher(Node):
    """
    ROS2 node that captures frames from a webcam and publishes them
    to the /cam_feed topic as sensor_msgs/Image messages.
    """
    def __init__(self):
        super().__init__('webcam_publisher')
        
        # Create publisher for image feed using the Image message type
        self.publisher = self.create_publisher(
            Image,
            '/cam_feed',
            10
        )
        
        # Create parameters
        self.declare_parameter('camera_id', 0)  # Default to first camera
        self.declare_parameter('frame_rate', 10.0)  # FPS
        self.declare_parameter('resolution_width', 640)
        self.declare_parameter('resolution_height', 480)
        
        # Get parameters
        self.camera_id = self.get_parameter('camera_id').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.width = self.get_parameter('resolution_width').value
        self.height = self.get_parameter('resolution_height').value
        
        # Initialize OpenCV->ROS bridge
        self.bridge = CvBridge()
        
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
        """Capture a frame and publish it to the /cam_feed topic as an Image message."""
        try:
            # Capture frame
            ret, frame = self.cap.read()
            
            if not ret:
                self.get_logger().error('Failed to capture frame from webcam')
                return
            
            # Convert to RGB for consistency (OpenCV uses BGR by default)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
            
            # Set frame timestamp
            ros_image.header.stamp = self.get_clock().now().to_msg()
            # Optionally set a frame_id if needed
            ros_image.header.frame_id = "camera_frame"
            
            # Publish the image
            self.publisher.publish(ros_image)
            
            self.get_logger().debug('Published frame to /cam_feed')
            
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