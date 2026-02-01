#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import numpy as np 
import os

import cv2

import cv_bridge

class ImageProcessing(Node):
    def __init__(self):
        super().__init__('mola_apriltag_estimation')

        # Initialize the TransformBroadcaster
        self.apiltag_image_publisher = self.create_publisher(Image, 'mola_auv/estimation/apriltag', 10)
        
        # Subscriptions
        self.subscription_thruster_state = self.create_subscription(
            Image,
            '/mola_auv/camera/image_color',
            self.compressed_image_callback,
            10
        )

        self.get_logger().info('MOLA AprilTag Estimation Node Initialized')
    

    def compressed_image_callback(self, mola_image_msg:Image):
        bridge = cv_bridge.CvBridge()

        apriltag_img_msg = Image()

        cv_image = bridge.imgmsg_to_cv2(mola_image_msg, desired_encoding='bgr8')

        if cv_image is None:
            self.get_logger().warn("Failed to encode image")
            return

        # Convert the OpenCV image back into a sensor_msgs/Image
        apriltag_img_msg = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        apriltag_img_msg.header = mola_image_msg.header

        self.apiltag_image_publisher.publish(apriltag_img_msg)


    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessing()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
