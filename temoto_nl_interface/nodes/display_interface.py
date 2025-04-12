#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
import json
import base64
import cv2
import numpy as np
import time
import threading

class Feed:
    """
    Class to handle image data processing and encoding for a specific feed.
    """
    def __init__(self, name, target, node_logger):
        self.name = name
        self.target = target
        self.key = f"{target}/{name}"
        self.cv_bridge = CvBridge()
        self.node_logger = node_logger
        self.last_publish_time = 0.0
        self.supported_encodings = {
            'rgb8', 'rgba8', 'bgr8', 'mono8', 'mono16', 
            'bayer_rggb8', 'bayer_bggr8', 'bayer_gbrg8', 'bayer_grbg8'
        }
        
    def update_last_publish_time(self):
        """Update the last time this feed was published"""
        self.last_publish_time = time.time()
        
    def process_image(self, image_data, jpeg_quality=85):
        """Process image data and convert to OpenCV format"""
        try:
            # Determine if this is a base64 string or a raw image dict
            if isinstance(image_data, str):
                # Assume base64 encoded image
                img_bytes = base64.b64decode(image_data)
                np_arr = np.frombuffer(img_bytes, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                # Convert to RGB for consistency
                if cv_image is not None and len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            elif isinstance(image_data, dict):
                # Raw image data dict (assuming it has 'data', 'height', 'width', 'encoding')
                if not all(key in image_data for key in ['data', 'height', 'width', 'encoding']):
                    self.node_logger.error(f'Invalid image data format for {self.key}')
                    return None
                
                # Convert dict to CV image based on encoding
                encoding = image_data['encoding'].lower()
                height = image_data['height']
                width = image_data['width']
                
                # Convert byte string to numpy array
                if isinstance(image_data['data'], str):
                    # If data is base64 encoded
                    img_bytes = base64.b64decode(image_data['data'])
                else:
                    # If data is already bytes
                    img_bytes = image_data['data']
                
                # Create numpy array from bytes
                np_arr = np.frombuffer(img_bytes, np.uint8)
                
                # Reshape based on encoding
                if encoding == 'rgb8':
                    cv_image = np.reshape(np_arr, (height, width, 3))
                elif encoding == 'rgba8':
                    cv_image = np.reshape(np_arr, (height, width, 4))
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2RGB)
                elif encoding == 'bgr8':
                    cv_image = np.reshape(np_arr, (height, width, 3))
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                elif encoding in ['mono8', 'mono16']:
                    cv_image = np.reshape(np_arr, (height, width))
                elif encoding.startswith('bayer'):
                    cv_image = np.reshape(np_arr, (height, width))
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerRG2RGB)
                else:
                    self.node_logger.error(f'Unsupported encoding: {encoding}')
                    return None
            else:
                self.node_logger.error(f'Invalid image data type for {self.key}')
                return None
                
            return cv_image
            
        except Exception as e:
            self.node_logger.error(f'Error processing image {self.key}: {str(e)}')
            return None
            
    def create_compressed_image_msg(self, cv_image, jpeg_quality=85, timestamp=None):
        """Convert OpenCV image to JPEG and create a CompressedImage message"""
        if cv_image is None:
            self.node_logger.error(f'Cannot create message from None image for {self.key}')
            return None
            
        try:
            # Encode image as JPEG
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
            
            # Create CompressedImage message
            compressed_msg = CompressedImage()
            
            # Set header with timestamp and camera name
            compressed_msg.header.stamp = timestamp if timestamp else rclpy.clock.Clock().now().to_msg()
            compressed_msg.header.frame_id = self.key
            
            compressed_msg.format = 'jpeg'
            compressed_msg.data = buffer.tobytes()
            
            return compressed_msg
        except Exception as e:
            self.node_logger.error(f'Error creating compressed image message for {self.key}: {str(e)}')
            return None


class FeedManager:
    """
    Thread-safe manager class to handle multiple image feeds
    """
    def __init__(self, node_logger):
        self.feeds = {}
        self.lock = threading.RLock()
        self.node_logger = node_logger
        
    def get_feed(self, target, name):
        """Get or create a feed for the given target and name"""
        with self.lock:
            key = f"{target}/{name}"
            if key not in self.feeds:
                self.feeds[key] = Feed(name, target, self.node_logger)
                self.node_logger.info(f"Created new feed: {key}")
            return self.feeds[key]
            
    def get_all_feeds(self):
        """Get all feeds as a dictionary"""
        with self.lock:
            return {key: feed for key, feed in self.feeds.items()}
            
    def get_feed_by_key(self, key):
        """Get a feed by key"""
        with self.lock:
            return self.feeds.get(key)
            
    def delete_feed(self, key):
        """Delete a feed"""
        with self.lock:
            if key in self.feeds:
                del self.feeds[key]
                self.node_logger.info(f"Deleted feed: {key}")
                return True
            return False


class FeedInterfaceNode(Node):
    """
    ROS2 node that standardizes image data from various formats
    and publishes them for the temoto assistance web interface.
    """
    def __init__(self):
        super().__init__('feed_interface_node')
        
        # Create callback groups
        self.subscription_callback_group = ReentrantCallbackGroup()
        
        # Initialize parameters
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('min_publish_interval', 0.1)  # seconds
        
        # Initialize feed manager
        self.feed_manager = FeedManager(self.get_logger())
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Dictionary to store image publishers
        self.image_publishers = {}
        
        # Create subscription for JSON-formatted input
        self.json_subscription = self.create_subscription(
            String,
            '/display_feed',
            self.media_request_callback,
            10,
            callback_group=self.subscription_callback_group
        )
        
        self.get_logger().info('Feed Interface Node is active')

    def media_request_callback(self, msg):
        """Process incoming JSON-formatted media requests with image data."""
        try:
            # Parse JSON input
            input_data = json.loads(msg.data)
            
            # Validate required fields
            if not all(key in input_data for key in ['target', 'name', 'image']):
                self.get_logger().error('Invalid message format: missing required fields (target, name, image)')
                return
                
            target = input_data['target']
            name = input_data['name']
            image_data = input_data['image']
            
            # Get or create feed for this target/name
            feed = self.feed_manager.get_feed(target, name)
            
            # Check if we should publish (rate limiting)
            if not self.should_publish(feed):
                return
                
            # Process the image data
            self.process_and_publish_image(feed, image_data)
                
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse JSON message')
        except Exception as e:
            self.get_logger().error(f'Unexpected error in media request callback: {str(e)}')
            
    def should_publish(self, feed):
        """Check if we should publish based on rate limiting"""
        min_interval = self.get_parameter('min_publish_interval').value
        current_time = time.time()
        
        if (current_time - feed.last_publish_time) < min_interval:
            return False
            
        return True
        
    def process_and_publish_image(self, feed, image_data):
        """Process image data and publish to webapp"""
        # Get JPEG quality parameter
        jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # Process the image
        cv_image = feed.process_image(image_data, jpeg_quality)
        if cv_image is None:
            return
            
        # Create the compressed image message
        compressed_msg = feed.create_compressed_image_msg(
            cv_image, 
            jpeg_quality,
            self.get_clock().now().to_msg()
        )
        if compressed_msg is None:
            return
            
        # Create a unique key for this feed
        publisher_key = feed.key
        
        # Create dedicated publisher for this feed if it doesn't exist
        if publisher_key not in self.image_publishers:
            # Create publisher for compressed image
            self.image_publishers[publisher_key] = self.create_publisher(
                CompressedImage,
                f'/webapp/display_feed/{feed.target}/{feed.name}',
                10
            )
            self.get_logger().info(f'Created new publisher for {publisher_key}')
        
        # Publish the compressed image
        self.image_publishers[publisher_key].publish(compressed_msg)
        
        # Update last publish time
        feed.update_last_publish_time()
        self.get_logger().debug(f'Published image for {publisher_key}')


def main(args=None):
    rclpy.init(args=args)
    
    executor = MultiThreadedExecutor()
    node = FeedInterfaceNode()
    
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped via keyboard interrupt.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()