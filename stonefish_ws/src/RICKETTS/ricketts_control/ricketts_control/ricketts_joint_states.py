#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from stonefish_ros2.msg import INS
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from stonefish_ros2.msg import ThrusterState

class RickettsJointStates(Node):
    def __init__(self):
        super().__init__('ricketts_joint_states')
        
        self.thruster_names = [
            "base_propLFU", 
            "base_propLFD",
            "base_propRFU",
            "base_propRFD",
            "base_propLBU",
            "base_propLBD",
            "base_propRBU",
            "base_propRBD"
        ]

        # Initialize the TransformBroadcaster
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscriptions
        self.subscription_thruster_state = self.create_subscription(
            ThrusterState,
            '/ricketts/controller/thruster_state',
            self.callback_thruster_state,
            10
        )
        self.subscription_lightL_state = self.create_subscription(
            Float64,
            '/ricketts/servo/lightL',
            self.callback_lightL_servo_state,
            10
        )
        self.subscription_lightR_state = self.create_subscription(
            Float64,
            '/ricketts/servo/lightR',
            self.callback_lightR_servo_state,
            10
        )

        joints_timer_period = 0.01  # seconds
        self.joints_timer = self.create_timer(joints_timer_period, self.joint_update_callback)

        self.joint_names =  [
            'base_lightL', 'base_lightR',
            'base_propLFU', 'base_propLFD',
            'base_propRFU', 'base_propRFD',
            'base_propLBU', 'base_propLBD',
            'base_propRBU', 'base_propRBD'
        ]

        self.joint_position = [
            0.0,   # base_lightL
            0.0,  # base_lightR
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  # all propellers
        ]
        self.joint_velocity = [0.0 for i in range(len(self.joint_names))]
        self.joint_effort = [0.0 for i in range(len(self.joint_names))]

        self.joint_states = {'base_propLFU': [0.0, 0.0, 0.0],
                             'base_propLFD': [0.0, 0.0, 0.0],
                             'base_propRFU': [0.0, 0.0, 0.0],
                             'base_propRFD': [0.0, 0.0, 0.0],
                             'base_propLBU': [0.0, 0.0, 0.0],
                             'base_propLBD': [0.0, 0.0, 0.0],
                             'base_propRBU': [0.0, 0.0, 0.0],
                             'base_propRBD': [0.0, 0.0, 0.0],
                             'base_lightL':  [0.0, 0.0, 0.0],
                             'base_lightR':  [0.0, 0.0, 0.0],
        }
    

    def joint_update_callback(self):
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = list(self.joint_states.keys())
        joint_msg.position = [state[0] for state in self.joint_states.values()]
        joint_msg.velocity = [state[1] for state in self.joint_states.values()]
        joint_msg.effort   = [state[2] for state in self.joint_states.values()]

        self.joint_publisher.publish(joint_msg)

    def callback_thruster_state(self, msg:ThrusterState):
        # Create TransformStamped message
        joint_names = list(self.joint_states.keys())
        joint_names = joint_names[:8]
        for joint_name in joint_names:
            self.joint_states[joint_name] = [0.0, 0.0, 0.0]



    def callback_lightL_servo_state(self, msg:Float64):
        joint_name = 'base_lightL'
        self.joint_states[joint_name] = [msg.data, 0.0, 0.0]


    def callback_lightR_servo_state(self, msg:Float64):
        joint_name = 'base_lightR'
        self.joint_states[joint_name] = [msg.data, 0.0, 0.0]


def main(args=None):
    rclpy.init(args=args)
    node = MolaJointStates()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
