#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv_bridge
import os


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        # Declare parameters
        self.declare_parameter('image_path', '')
        self.image_path = self.get_parameter(
            'image_path').get_parameter_value().string_value

        if not self.image_path or not os.path.exists(self.image_path):
            self.get_logger().error('Image path not set or invalid!')
            rclpy.shutdown()
            return

        # List all filenames in the image_path directory
        filenames = os.listdir(self.image_path)
        self.get_logger().info(
            f'Files in directory "{self.image_path}": {filenames}')

        # Filter for files in the format image_*.jpg
        self.image_files = [f for f in filenames if f.startswith(
            'image_') and f.endswith('.jpg')]
        self.get_logger().info(
            f'Filtered image files: {self.image_files}')

        if not self.image_files:
            self.get_logger().error('No valid image files found!')
            rclpy.shutdown()
            return

        # Subscribe to the images topic
        self.subscription = self.create_subscription(
            Image,
            'images',
            self.image_callback,
            10
        )

        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.bridge = cv_bridge.CvBridge()
        self.current_index = 0

        # Publish the first image
        self.publish_image()

    def publish_image(self):

        if self.current_index >= len(self.image_files):
            self.get_logger().info('All images published. Shutting down.')
            raise SystemExit

        image_path = os.path.join(
            self.image_path, self.image_files[self.current_index])
        self.get_logger().info(f'Publishing image: {image_path}')

        image = cv2.imread(image_path)
        if image is None:
            self.get_logger().error(f'Failed to load image: {image_path}')
            rclpy.shutdown()
            return

        msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published image: {self.image_files[self.current_index]}')

    def image_callback(self, msg):
        self.get_logger().info('Received image, publishing next one.')
        self.current_index += 1
        self.publish_image()


def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    try:
        rclpy.spin(video_publisher)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    video_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
