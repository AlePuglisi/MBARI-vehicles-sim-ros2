#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy\

from ament_index_python.packages import get_package_share_directory

import numpy as np 
import os

import cv2
from cv_bridge import CvBridge

from ultralytics import YOLO

# For now hardcoded model name 
MODEL_NAME = "mbari-megalodon-yolov8x.pt"

class SpeciesDetection(Node):
    def __init__(self):
        super().__init__('mola_species_detection')

        # Initialize the TransformBroadcaster
        self.species_bbox_image_publisher = self.create_publisher(Image, 'mola_auv/species_detection/detection', 10)

        # image_qos = QoSProfile(
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=5,
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        # )

        # Subscriptions
        self.subscription_forward_image = self.create_subscription(
            Image,
            '/mola_auv/front_camera/image_color',
            self.camera_image_callback,
            10
        )

        self.camera_params = None
        self.subscription_camera_info = self.create_subscription(
            CameraInfo,
            '/mola_auv/front_camera/camera_info',
            self.camera_info_callback,
            10
        )

        self._cv_bridge = CvBridge()

        # Initialize YOLO detection model FathomNet
        self._init_detector()

        self.get_logger().info('MOLA Species Detection Node Initialized')
    
    def _init_detector(self):
        description_path = get_package_share_directory('mola_auv_estimation')
        model_path = os.path.join(description_path, "models", MODEL_NAME)

        self.model = YOLO(model_path)

        self.get_logger().info('YOLO Detection Model Initialized')

    def camera_info_callback(self, msg: CameraInfo):
        """Extract camera parameters from CameraInfo message"""
        if self.camera_params is None:
            # K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.camera_params = [
                msg.k[0],  # fx
                msg.k[4],  # fy
                msg.k[2],  # cx
                msg.k[5]   # cy
            ]
            self.get_logger().info(f"Camera intrinsics received: {self.camera_params}")


    def camera_image_callback(self, mola_image_msg:Image):

        species_detection_img_msg = Image()

        try: 
            cv_image = self._cv_bridge.imgmsg_to_cv2(mola_image_msg, desired_encoding='bgr8')
        except Exception as e: 
            self.get_logger().error(f"Error converting image: {e}")
            return
        
        # Downsample for detection
        # scale_factor = 0.5
        # cv_image = cv2.resize(cv_image, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_AREA)

        try:
            # predict returns a list with one Results object per input image
            detection_results = self.model.predict(
                cv_image,
                conf=0.25,
                verbose=False,
            )

            # Annotated image as a BGR numpy array
            annotated_image = detection_results[0].plot()

            species_detection_img_msg = self._cv_bridge.cv2_to_imgmsg(
                annotated_image, encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f"Error running detection: {e}")
            return

        # Keep the original timestamp and frame so it lines up with the camera data
        species_detection_img_msg.header = mola_image_msg.header

        self.species_bbox_image_publisher.publish(species_detection_img_msg)



    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeciesDetection()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
