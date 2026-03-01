#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class RickettsJointStates(Node):
    def __init__(self):
        super().__init__('ricketts_joint_states')

        # Initialize the TransformBroadcaster
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        self.subscription_front_camera_link_state = self.create_subscription(
            Float64,
            'ricketts/servo/base_link_camera',
            self.callback_base_link_camera_servo_state,
            10
        )
        self.subscription_front_camera_state = self.create_subscription(
            Float64,
            'ricketts/servo/link_camera',
            self.callback_link_camera_servo_state,
            10
        )
        self.subscription_stereo_camera_link_state = self.create_subscription(
            Float64,
            'ricketts/servo/base_link_stereo',
            self.callback_base_link_stereo_servo_state,
            10
        )
        self.subscription_stereo_camera_state = self.create_subscription(
            Float64,
            'ricketts/servo/link_stereo',
            self.callback_link_stereo_servo_state,
            10
        )

        joints_timer_period = 0.01  # seconds
        self.joints_timer = self.create_timer(joints_timer_period, self.joint_update_callback)

        self.joint_names =  [
            'frame_front_camera_link', 'front_camera_link_camera', 
            'frame_stereo_camera_link', 'stereo_camera_link_stereo_camera'
        ]

        self.joint_position = [
            0.0,   # frame_front_camera_link
            0.0,   # front_camera_link_camera
            0.0,   # frame_stereo_camera_link
            0.0    # stereo_camera_link_stereo_camera
        ]

        self.joint_velocity = [0.0 for i in range(len(self.joint_names))]
        self.joint_effort = [0.0 for i in range(len(self.joint_names))]

        self.joint_states = {
                             'frame_front_camera_link':  [0.0, 0.0, 0.0],
                             'front_camera_link_camera':  [0.0, 0.0, 0.0],
                             'frame_stereo_camera_link':  [0.0, 0.0, 0.0],
                             'stereo_camera_link_stereo_camera':  [0.0, 0.0, 0.0]
        }
    

    def joint_update_callback(self):
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = list(self.joint_states.keys())
        joint_msg.position = [state[0] for state in self.joint_states.values()]
        joint_msg.velocity = [state[1] for state in self.joint_states.values()]
        joint_msg.effort   = [state[2] for state in self.joint_states.values()]

        self.joint_publisher.publish(joint_msg)


    def callback_base_link_camera_servo_state(self, msg:Float64):
        joint_name = 'frame_front_camera_link'
        self.joint_states[joint_name] = [msg.data, 0.0, 0.0]


    def callback_link_camera_servo_state(self, msg:Float64):
        joint_name = 'front_camera_link_camera'
        self.joint_states[joint_name] = [msg.data, 0.0, 0.0]

    def callback_base_link_stereo_servo_state(self, msg:Float64):
        joint_name = 'frame_stereo_camera_link'
        self.joint_states[joint_name] = [msg.data, 0.0, 0.0]


    def callback_link_stereo_servo_state(self, msg:Float64):
        joint_name = 'stereo_camera_link_stereo_camera'
        self.joint_states[joint_name] = [msg.data, 0.0, 0.0]

def main(args=None):
    rclpy.init(args=args)
    node = RickettsJointStates()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
