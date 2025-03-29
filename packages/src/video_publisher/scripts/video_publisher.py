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
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # 10Hz
        self.bridge = cv_bridge.CvBridge()

        # Declare parameters
        self.declare_parameter('image_path', '')
        self.image_path = self.get_parameter(
            'image_path').get_parameter_value().string_value

        if not self.image_path or not os.path.exists(self.image_path):
            self.get_logger().error('Image path not set or invalid!')
            rclpy.shutdown()
            return

        self.image = cv2.imread(self.image_path)
        if self.image is None:
            self.get_logger().error('Failed to load image!')
            rclpy.shutdown()
            return

        self.get_logger().info('Setup complete, publishing video frames...')

    def timer_callback(self):
        msg = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
