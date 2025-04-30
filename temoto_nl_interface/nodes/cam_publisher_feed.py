#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import json
import os
import base64
from pathlib import Path
import time
import glob

class DesktopImagePublisherNode(Node):
    """
    Node that publishes three specific images from Desktop to display_feed
    with the specified actor and feed combinations.
    """
    def __init__(self):
        super().__init__('desktop_image_publisher')
        # Publisher for display_feed
        self.display_feed_publisher = self.create_publisher(
            String,
            '/display_feed',
            10
        )
        
        # Image configurations - now with common image extensions
        self.image_configs = [
            {"path": "image_1", "actor": "David", "feed": "Camera"},
            {"path": "image_2", "actor": "David", "feed": "rviz"},
            {"path": "image_3", "actor": "John", "feed": "Camera"}
        ]
        
        # Get desktop path
        self.desktop_path = str(Path.home() / "Desktop")
        self.get_logger().info(f"Looking for images in: {self.desktop_path}")
        
        # List all files in the desktop directory
        self.list_desktop_files()
        
        # Setup a timer to publish images
        self.timer = self.create_timer(1.0, self.publish_images)
        self.get_logger().info("Desktop Image Publisher Node started")
    
    def list_desktop_files(self):
        """List all files in the desktop directory to help debugging"""
        try:
            files = os.listdir(self.desktop_path)
            self.get_logger().info(f"Files in desktop directory: {files}")
        except Exception as e:
            self.get_logger().error(f"Error listing desktop files: {str(e)}")
    
    def find_image_with_extensions(self, base_name):
        """Try to find an image file with common extensions"""
        # Common image extensions to try
        extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.gif']
        
        # First try exact match
        if os.path.exists(os.path.join(self.desktop_path, base_name)):
            return base_name
        
        # Then try with extensions
        for ext in extensions:
            full_path = os.path.join(self.desktop_path, base_name + ext)
            if os.path.exists(full_path):
                return base_name + ext
        
        # Also try case-insensitive match
        desktop_files = os.listdir(self.desktop_path)
        base_name_lower = base_name.lower()
        for file in desktop_files:
            if file.lower().startswith(base_name_lower):
                # Check if it's likely an image file
                if any(file.lower().endswith(ext) for ext in extensions):
                    return file
        
        return None
    
    def read_image(self, image_base_name):
        """Read an image from the desktop and convert to base64"""
        # Try to find the actual image file with appropriate extension
        actual_filename = self.find_image_with_extensions(image_base_name)
        
        if not actual_filename:
            self.get_logger().error(f"Image not found for base name: {image_base_name}")
            return None
        
        image_path = os.path.join(self.desktop_path, actual_filename)
        try:
            # Read the image
            img = cv2.imread(image_path)
            if img is None:
                self.get_logger().error(f"Failed to read image: {image_path}")
                return None
                
            # Convert to base64
            _, buffer = cv2.imencode('.jpg', img)
            base64_image = base64.b64encode(buffer).decode('utf-8')
            self.get_logger().info(f"Successfully read image: {image_path}")
            return base64_image
        except Exception as e:
            self.get_logger().error(f"Error reading image {image_path}: {str(e)}")
            return None
    
    def publish_images(self):
        """Publish all images to display_feed topic"""
        for config in self.image_configs:
            image_base64 = self.read_image(config["path"])
            if image_base64 is not None:
                # Create message payload
                payload = {
                    "target": config["actor"],
                    "name": config["feed"],
                    "image": image_base64
                }
                
                # Convert to JSON string
                msg = String()
                msg.data = json.dumps(payload)
                
                # Publish
                self.display_feed_publisher.publish(msg)
                self.get_logger().info(f"Published image {config['path']} for Actor {config['actor']} with feed {config['feed']}")
            
            # Small delay to prevent overwhelming the system
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = DesktopImagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped via keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()