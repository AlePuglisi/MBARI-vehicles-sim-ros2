#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import numpy as np 
import os

import cv2
import apriltag 

from cv_bridge import CvBridge

class ImageProcessing(Node):
    def __init__(self):
        super().__init__('mola_apriltag_estimation')

        # Initialize the TransformBroadcaster
        self.apiltag_image_publisher = self.create_publisher(Image, 'mola_auv/estimation/apriltag', 10)
        
        # Subscriptions
        self.subscription_thruster_state = self.create_subscription(
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

        # Tag size in meters
        self.tag_size = 0.160

        apriltag_options = apriltag.DetectorOptions(families='tag25h9', 
                                                    border=1,
                                                    nthreads=4,
                                                    quad_decimate=2.0,        # Downsample by 2x (reduces contours)
                                                    quad_blur=0.8,            # Apply blur to reduce noise
                                                    refine_edges=True,
                                                    debug=False) 
        
        self.apriltag_detector = apriltag.Detector(apriltag_options)


        self.get_logger().info('MOLA AprilTag Estimation Node Initialized')
    

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

        apriltag_img_msg = Image()

        try: 
            cv_image = self._cv_bridge.imgmsg_to_cv2(mola_image_msg, desired_encoding='bgr8')
        except Exception as e: 
            self.get_logger().error(f"Error converting image: {e}")
            return
        
        # Downsample for detection
        # scale_factor = 0.5
        # cv_image = cv2.resize(cv_image, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_AREA)
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        apriltag_results = self.apriltag_detector.detect(gray_img)


        for r in apriltag_results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(cv_image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(cv_image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(cv_image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(cv_image, ptD, ptA, (0, 255, 0), 2)

            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)

            # draw the tag family on the image
            tagId = str(r.tag_id)
            cv2.putText(cv_image, tagId, (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 2)
            
            # Access pose estimation results
            # Compute pose if camera parameters are available
            if self.camera_params is not None:
                pose_matrix, init_error, final_error = self.apriltag_detector.detection_pose(
                    r,
                    self.camera_params,
                    tag_size=self.tag_size,
                    z_sign=1
                )
                
                # pose_matrix is a 4x4 transformation matrix
                self._draw_pose_axes(cv_image, pose_matrix, self.camera_params, self.tag_size)


        # Convert the OpenCV image back into a sensor_msgs/Image
        apriltag_img_msg = self._cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        apriltag_img_msg.header = mola_image_msg.header

        self.apiltag_image_publisher.publish(apriltag_img_msg)

    def _draw_pose_axes(self, image, pose_matrix, camera_params, tag_size):
        """Draw 3D coordinate axes on the detected tag"""
        
        # Define 3D points for the axes (in tag frame)
        # Origin at tag center, axes extend by half the tag size
        axis_length = tag_size / 2
        opoints = np.array([
            [0, 0, 0],              # origin
            [axis_length, 0, 0],    # x-axis point
            [0, axis_length, 0],    # y-axis point
            [0, 0, -axis_length]    # z-axis point (into camera)
        ]).reshape(-1, 1, 3)
        
        # Camera intrinsic matrix
        fx, fy, cx, cy = camera_params
        K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])
        
        # Extract rotation and translation from pose matrix
        rvec, _ = cv2.Rodrigues(pose_matrix[:3, :3])
        tvec = pose_matrix[:3, 3]
        
        # Project 3D points to image plane
        dcoeffs = np.zeros(5)  # no distortion
        ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)
        ipoints = np.round(ipoints).astype(int).reshape(-1, 2)
        
        # Draw axes
        origin = tuple(ipoints[0])
        cv2.line(image, origin, tuple(ipoints[1]), (0, 0, 255), 3)    # X-axis (red)
        cv2.line(image, origin, tuple(ipoints[2]), (0, 255, 0), 3)    # Y-axis (green)
        cv2.line(image, origin, tuple(ipoints[3]), (255, 0, 0), 3)    # Z-axis (blue)

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
