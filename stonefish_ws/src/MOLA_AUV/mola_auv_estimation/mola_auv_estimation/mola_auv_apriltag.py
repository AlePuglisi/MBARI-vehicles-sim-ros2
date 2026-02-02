import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

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
            '/mola_auv/camera/image_color',
            self.compressed_image_callback,
            10
        )

        self._cv_bridge = CvBridge()

        apriltag_options = apriltag.DetectorOptions(families='tag36h11')
        self.apriltag_detector = apriltag.Detector(apriltag_options)


        self.get_logger().info('MOLA AprilTag Estimation Node Initialized')
    

    def compressed_image_callback(self, mola_image_msg:Image):

        apriltag_img_msg = Image()

        try: 
            cv_image = self._cv_bridge.imgmsg_to_cv2(mola_image_msg, desired_encoding='bgr8')
        except Exception as e: 
            self.get_logger().error(f"Error converting image: {e}")
            return

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
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Convert the OpenCV image back into a sensor_msgs/Image
        apriltag_img_msg = self._cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
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
