#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from stonefish_ros2.msg import INS
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomToTFNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        
        # Initialize the TransformBroadcaster
        self.br = TransformBroadcaster(self)
        
        # Subscriptions
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/ricketts/navigator/odometry',
            self.callback_odom,
            10
        )
        self.subscription_ins = self.create_subscription(
            INS,
            '/ricketts/navigator/INS',
            self.callback_ins,
            10
        )
    
    def callback_odom(self, msg):
        # Create TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world_ned'
        t.child_frame_id = 'odom'
        
        # Set translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Set rotation
        t.transform.rotation = msg.pose.pose.orientation
        
        # Broadcast the transform
        self.br.sendTransform(t)
    
    def callback_ins(self, msg):
        # Calculate quaternion from roll, pitch, yaw
        rotation = R.from_euler('xyz', [-msg.pose.roll, msg.pose.pitch, msg.pose.yaw])
        q = rotation.as_quat()  # Returns [x, y, z, w] in scipy

        # Create TransformStamped message
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world_ned'
        t.child_frame_id = 'odom'
        
        # Set translation
        t.transform.translation.x = msg.pose.north
        t.transform.translation.y = msg.pose.east
        t.transform.translation.z = msg.pose.down
        
        # Set rotation
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Broadcast the transform
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTFNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
