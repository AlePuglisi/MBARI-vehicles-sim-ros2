#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from math import pi

import numpy as np
from std_srvs.srv import SetBool

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os 

class WeightedJoystickController(Node):
    """ Joystick controller """ 

    def __init__(self):
        super().__init__('weighted_joystick_controller')
        
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.img_sub = self.create_subscription(
            Image,
            '/mola_auv/camera/image_color',
            self.image_callback,
            10)
 
        self.cv_bridge = CvBridge() 
        self.img_count = 1
        self.cv_image = None
        self.img_path = "/home/ale/tutorials/stonefish-learning/stonefish_ws/src/MBARI/MOLA_AUV/mola_auv_sim/data/images/"

        self.rov_control_pub = self.create_publisher(
            Float64MultiArray, 
            '/mola_auv/controller/thruster_setpoints_sim', 
            10
        )

        self.mola_lightL_servo_pub = self.create_publisher(
            Float64, 
            '/mola_auv/servo/lightL', 
            10
        )

        self.lightL_switch_cli = self.create_client(SetBool, '/mola_auv/lights/left')

        self.mola_lightR_servo_pub = self.create_publisher(
            Float64, 
            '/mola_auv/servo/lightR', 
            10
        )

        self.lightR_switch_cli = self.create_client(SetBool, '/mola_auv/lights/right')

        # Wait for services to be available
        while not self.lightL_switch_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for left light service...')
        while not self.lightR_switch_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for right light service...')
        
        self.get_logger().info('Both light services are available')

        # Track light state for toggling
        self.lights_on = False
        self.share_button_pressed = False  # For edge detection

        # Joystick mapping
        self.forward_joy = 1
        self.side_joy = 0
        self.depth_joy = 4
        self.yaw_joy = 3
        self.pitch_joy = 7
        self.roll_joy = 6

        # PS4 Joypad 
        # Axes Index        
        self.axes_index_ = {
            "analog_left_LR":  0,   # left analog stick        ( Left = +1 | Right = -1 )
            "analog_left_UD":  1,   # left analog stick        ( Up   = +1 | Down  = -1 )
            "analog_right_LR": 3,   # right analog stick       ( Left = +1 | Right = -1 )
            "analog_right_UD": 4,   # right analog stick       ( Up   = +1 | Down  = -1 )
            "arrow_LR": 6,          # arrow buttons left/right ( Left = +1 | Right = -1 )
            "arrow_UD": 7           # arrow buttons up/down    ( Up   = +1 | Down  = -1 )
        }

        # Buttons Index 
        self.buttons_index_ = {
            "x":        0,
            "circle":   1,
            "triangle": 2,
            "square":   3,
            "L1":       4,
            "R1":       5,  
            "L2":       6,
            "R2":       7,
            "share":    8,
            "options":  9,
            "ps":       10,
            "L3":       11,
            "R3":       12
        }

        self.deadzone = 0.05
        
        self.servo_state = 0.0

        # Build allocation matrix and compute weighted pseudo-inverse
        self.build_allocation_matrix()
        
        self.get_logger().info('Weighted joystick controller initialized')
        self.get_logger().info(f'Using inertia-based weighting')

    def build_allocation_matrix(self):
        """Build 6x8 thruster allocation matrix"""
        
        # Thruster configuration: [x, y, z, pitch_deg, yaw_deg]
        thrusters = [
            [0.3, -0.182, -0.105, -45, -135],  # T1_LFU
            [0.3, -0.182, 0.105,  45, -135],  # T2_LFD
            [0.3, 0.182, -0.105, -45, 135],  # T3_RFU
            [0.3, 0.182, 0.105, 45, 135],  # T4_RFD
            [-0.3, -0.182, -0.105, 45, 135], # T5_LBU
            [-0.3, -0.182, 0.105, -45, 135], # T6_LBD
            [-0.3, 0.182, -0.105,  45, -135], # T7_RBU
            [-0.3, 0.182, 0.105, -45, -135], # T8_RBD
        ]
        
        B = np.zeros((6, 8))
        
        for i, (x, y, z, pitch_deg, yaw_deg) in enumerate(thrusters):
            pitch = np.radians(pitch_deg)
            yaw = np.radians(yaw_deg)
            
            # Thrust direction
            fx = np.cos(pitch) * np.cos(yaw)
            fy = np.cos(pitch) * np.sin(yaw)
            fz = np.sin(pitch)
            
            # Torque
            tx = y * fz - z * fy
            ty = z * fx - x * fz
            tz = x * fy - y * fx
            
            B[:, i] = [fx, fy, fz, tx, ty, tz]
        
        self.B = B
        
        self.M = np.array([ [-1, -1, -1, -1, -1, -1, -1, -1],
                            [-1, -1, 1, 1, 1, 1, -1, -1],
                            [-1, 1, -1, 1, 1, -1, 1, -1],
                            [1, -1, -1, 1, -1, 1, 1, -1],
                            [1, -1, 1, -1, 1, -1, 1, -1],
                            [-1, -1, 1, 1, -1, -1, 1, 1]])
        self.M = self.M * 0.3

        # YOUR ACTUAL INERTIA VALUES (from the simulation)
        mass = 28.273 # kg
        Ixx = 0.668  # Roll inertia (VERY LOW!)
        Iyy = 2.359  # Pitch inertia
        Izz = 2.478 # Yaw inertia
        
        # Create weighting matrix that normalizes by inertia
        # This makes commands of equal magnitude produce equal angular accelerations
        # W scales each DOF by sqrt(inertia) so that all DOFs are balanced
        
        # For forces, use mass
        # For torques, use moment of inertia
        W = np.diag([
            1.0 / np.sqrt(mass),      # Surge (force)
            1.0 / np.sqrt(mass),      # Sway (force)
            1.0 / np.sqrt(mass),      # Heave (force)
            1.0 / np.sqrt(Ixx),       # Roll (torque) - LARGE weight due to small Ixx
            1.0 / np.sqrt(Iyy),       # Pitch (torque)
            1.0 / np.sqrt(Izz)        # Yaw (torque)
        ])
        
        # Compute weighted pseudo-inverse: B_weighted^+ = B^T * W^2 * (B * B^T * W^2)^-1
        # Simpler form: (W*B)^+
        BW = W @ B

        # self.B_pinv = np.linalg.pinv(BW)
        self.B_pinv = np.linalg.pinv(self.M)
    

        # self.B_pinv = np.linalg.pinv(B)
        self.get_logger().info(f'Allocation matrix shape: {B.shape}')
        self.get_logger().info(f'Allocation matrix: {B}')
        # self.get_logger().info(f'Pseudo-inverse shape: {self.B_pinv.shape}')

        # Log the weighting ratios
        # self.get_logger().info(f'Weighting factors:')
        # self.get_logger().info(f'  Roll weight / Pitch weight = {np.sqrt(Iyy/Ixx):.2f}x')
        # self.get_logger().info(f'  Roll weight / Yaw weight = {np.sqrt(Izz/Ixx):.2f}x')
        # self.get_logger().info(f'This compensates for low roll inertia')

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def toggle_lights(self):
        """Toggle lights on/off by calling both service clients"""
        # Toggle state
        self.lights_on = not self.lights_on
        
        # Create service request
        request = SetBool.Request()
        request.data = self.lights_on
        
        # Call left light service asynchronously
        future_left = self.lightL_switch_cli.call_async(request)
        future_left.add_done_callback(self.light_left_callback)
        
        # Call right light service asynchronously
        future_right = self.lightR_switch_cli.call_async(request)
        future_right.add_done_callback(self.light_right_callback)
        
        self.get_logger().info(f'Toggling lights {"ON" if self.lights_on else "OFF"}')

    def light_left_callback(self, future):
        """Callback for left light service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Left light: {response.message}')
            else:
                self.get_logger().warn(f'Left light failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Left light service call failed: {str(e)}')

    def light_right_callback(self, future):
        """Callback for right light service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Right light: {response.message}')
            else:
                self.get_logger().warn(f'Right light failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Right light service call failed: {str(e)}')


    def image_callback(self, msg):
        try:
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return
        
        
    def joy_callback(self, data: Joy):
        """Callback for joystick commands"""
        axes = data.axes
        buttons = data.buttons
        
        # Extract commands
        surge = -self.apply_deadzone(-axes[self.forward_joy])
        sway = -self.apply_deadzone(axes[self.side_joy])
        heave = self.apply_deadzone(axes[self.depth_joy])
        roll = self.apply_deadzone(axes[self.roll_joy]) if self.roll_joy < len(axes) else 0.0
        pitch = -self.apply_deadzone(axes[self.pitch_joy]) if self.pitch_joy < len(axes) else 0.0
        yaw = -self.apply_deadzone(axes[self.yaw_joy])
        
        # Command vector [Surge, Sway, Heave, Roll, Pitch, Yaw]
        command_vector = np.array([surge, sway, heave, roll, pitch, yaw])
        
        # Calculate thruster setpoints using weighted pseudo-inverse
        # thruster_setpoints = self.B_pinv @ command_vector
        thruster_setpoints = self.M.transpose() @ command_vector


        max_thrust = np.max(np.abs(thruster_setpoints))
        if max_thrust > 0.4:
            thruster_setpoints =  thruster_setpoints / (max_thrust + 1.0)
        
        # Publish
        msg = Float64MultiArray()
        msg.data = thruster_setpoints.tolist()
        self.rov_control_pub.publish(msg)
        # self.get_logger().info(f"Thrusters SetPoints: {thruster_setpoints}")   


        if buttons[self.buttons_index_["R1"]] == 1:
            self.get_logger().info("R1 pressed")
            msg = Float64()
            if self.servo_state < pi/2:
                self.servo_state += 0.05
            msg.data = self.servo_state
            self.mola_lightL_servo_pub.publish(msg)

            msg.data = -self.servo_state
            self.mola_lightR_servo_pub.publish(msg)
        if buttons[self.buttons_index_["L1"]] == 1:
            self.get_logger().info("L1 pressed")
            msg = Float64()
            if self.servo_state > 0:
                self.servo_state -= 0.05
            msg.data = self.servo_state
            self.mola_lightL_servo_pub.publish(msg)
            
            msg.data = -self.servo_state
            self.mola_lightR_servo_pub.publish(msg)

        # Handle SHARE button for toggling lights (edge detection)
        share_button_state = buttons[self.buttons_index_["share"]] == 1
        if share_button_state and not self.share_button_pressed:
            # Button just pressed (rising edge)
            self.toggle_lights()
        self.share_button_pressed = share_button_state

        if buttons[self.buttons_index_["triangle"]] == 1 and self.cv_image is not None:
            # Button just pressed (rising edge)
            full_img_path = self.img_path + 'camera_' + str(self.img_count) + ".jpg"
            cv2.imwrite(full_img_path, self.cv_image)  
            self.img_count += 1


        # # Debug
        # if np.any(np.abs(command_vector) > 0.01):
        #     self.get_logger().info(
        #         f'CMD: Surge={surge:.2f} Sway={sway:.2f} Heave={heave:.2f} '
        #         f'Roll={roll:.2f} Pitch={pitch:.2f} Yaw={yaw:.2f} | Max thrust={max_thrust:.2f}'
        #     )


def main(args=None):
    rclpy.init(args=args)
    controller = WeightedJoystickController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()