#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from pathlib import Path

class StaticImagePublisher(Node):
    """
    ROS2 node that reads a static image file (demo.jpeg) from the desktop
    and publishes it to the /cam_feed topic as sensor_msgs/Image messages.
    """
    def __init__(self):
        super().__init__('static_image_publisher')
        
        # Create publisher for image feed using the Image message type
        self.publisher = self.create_publisher(
            Image,
            '/cam_feed',
            10
        )
        
        # Create parameters
        self.declare_parameter('publish_rate', 1.0)  # Default to 1 Hz (once per second)
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Initialize OpenCV->ROS bridge
        self.bridge = CvBridge()
        
        # Get path to desktop and image file
        self.desktop_path = str(Path.home() / "Desktop")
        self.image_path = os.path.join(self.desktop_path, "demo.jpeg")
        
        # Check if image file exists
        if not os.path.isfile(self.image_path):
            self.get_logger().error(f'Image file not found: {self.image_path}')
            raise FileNotFoundError(f"Could not find image file: {self.image_path}")
        
        # Load the static image
        self.static_image = cv2.imread(self.image_path)
        if self.static_image is None:
            self.get_logger().error(f'Failed to load image: {self.image_path}')
            raise RuntimeError(f"Could not load image: {self.image_path}")
        
        # Convert to RGB for consistency (OpenCV uses BGR by default)
        self.static_image_rgb = cv2.cvtColor(self.static_image, cv2.COLOR_BGR2RGB)
        
        # Get image dimensions
        self.height, self.width = self.static_image.shape[:2]
        self.get_logger().info(f'Loaded image with resolution: {self.width}x{self.height}')
        
        # Create timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Static image publisher initialized, publishing at {self.publish_rate} Hz')

    def timer_callback(self):
        """Publish the static image to the /cam_feed topic as an Image message."""
        try:
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(self.static_image_rgb, encoding="rgb8")
            
            # Set frame timestamp
            ros_image.header.stamp = self.get_clock().now().to_msg()
            # Set a frame_id
            ros_image.header.frame_id = "camera_frame"
            
            # Publish the image
            self.publisher.publish(ros_image)
            
            self.get_logger().debug('Published static image to /cam_feed')
            
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    static_image_publisher = StaticImagePublisher()
    
    try:
        rclpy.spin(static_image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        static_image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()