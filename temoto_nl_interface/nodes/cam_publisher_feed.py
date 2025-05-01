#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import json
import base64
from cv_bridge import CvBridge

class SpotImageSubscriberNode(Node):
    """
    Node that subscribes to Spot hand camera images and republishes them
    to display_feed with the specified format.
    """
    def __init__(self):
        super().__init__('spot_image_subscriber')
        
        # Publisher for display_feed
        self.display_feed_publisher = self.create_publisher(
            String,
            '/display_feed',
            10
        )
        
        # Subscriber for Spot hand camera images
        self.image_subscriber = self.create_subscription(
            Image,
            '/spot_image_server/rgb/hand_rgb/image',
            self.image_callback,
            10
        )
        
        # CV bridge for converting between ROS Image and OpenCV image
        self.cv_bridge = CvBridge()
        
        self.get_logger().info("Spot Image Subscriber Node started")
        self.get_logger().info("Subscribing to /spot_image_server/rgb/hand_rgb/image")
        self.get_logger().info("Publishing to /display_feed with format for 'David' and 'live_feed'")
    
    def image_callback(self, msg):
        """Callback for when an image is received"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert OpenCV image to base64
            _, buffer = cv2.imencode('.jpg', cv_image)
            base64_image = base64.b64encode(buffer).decode('utf-8')
            
            # Create message payload as per required format
            payload = {
                "target": "David",
                "name": "live_feed",
                "image": base64_image
            }
            
            # Convert to JSON string
            display_msg = String()
            display_msg.data = json.dumps(payload)
            
            # Publish
            self.display_feed_publisher.publish(display_msg)
            self.get_logger().info("Published Spot hand camera image to display_feed")
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SpotImageSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped via keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()