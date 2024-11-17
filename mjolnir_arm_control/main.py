# IceCube
# Main node for Etri Mjonir Arm control
#
# UiA Grimstad
# 25/9 - 24
#

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from mjolnir_arm_control.libserialservo import ServoController
from mjolnir_arm_control.libmjolnirarmcontrol import MjolnirArmControl
from mjolnir_arm_control.libmjolnirinvctrl import RoboticArmIK

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        # Declare and get parameters
        self.declare_parameter('serial_port', '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5713008361-if00')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('cmd_vel_topic', '/ArmTwist')
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value

        # Check for debug
        self.debug_log = self.get_env_as_bool('DEBUG_LOG', False)
        if self.debug_log:
            self.get_logger().info("Debug log enabled")

        self.dead_band = 0.01
        self.tool_vel =  np.array([ 0.1, 0.0, 0.1, 0.0, 0.0, 0.0])

        # Initialize the servo controller
        self.servo_controller = ServoController(port=serial_port, baud_rate=baud_rate)
        try:
            self.servo_controller.connect()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to servo controller: {e}")
            self.destroy_node()
            return

        self.servo_controller.send_servo_values([90,90,90,90,90,90])
        self.get_logger().info(f"Sent initial position to arm controller on port {serial_port}")

        self.arm_control = MjolnirArmControl()
        joint_limits = {
            'theta1': (-90, 90),
            'theta2': (-90, 90),
            'theta3': (0, 180),
        }
        #self.arm_inv_control = RoboticArmIK(0.065, 0.35, 0.304, joint_limits) #
        self.theta1 = 90.0  # <- Servo 1
        self.theta2 = 0.0   # <- DC-motor extender
        self.theta3 = 90.0  # <- Servo 2
        self.theta4 = 90.0  # <- ?

        # Subscribe to the Twist topic
        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.twist_callback,
            10
        )

        self.get_logger().info(f"Servo node initialized. Listening to {self.cmd_vel_topic}")

    def get_env_as_bool(self, env_var, default_value):
        value = os.getenv(env_var, None)

        if value is None:
            return default_value  # Environment variable is not set, return default

        # Try to convert the value to a boolean
        try:
            return value.lower() in ['1', 'true', 'yes', 'on']
        except AttributeError:
            return default_value  # Value is not a string, fallback to default

    def twist_callback(self, msg):
        # Process the Twist message and compute servo positions
        # Map velocities to servo positions

        #print(f"This is self.dead_band < msg.linear.x = {self.dead_band < msg.linear.x}")

        # Map linear velocities to servos 1-3
        if self.dead_band < msg.linear.x or msg.linear.x < -self.dead_band:
            self.tool_vel[0] = msg.linear.x
        else:
            self.tool_vel[0] = 0

        if self.dead_band < msg.linear.y or msg.linear.y < -self.dead_band:
            self.tool_vel[1] = msg.linear.y
        else:
            self.tool_vel[1] = 0

        if self.dead_band < msg.linear.z or msg.linear.z < -self.dead_band:
            self.tool_vel[2] = msg.linear.z
        else:
            self.tool_vel[2] = 0

        # Map angular velocities to servos 4-6
        self.tool_vel[3] = msg.angular.x
        self.tool_vel[4] = msg.angular.y
        self.tool_vel[5] = msg.angular.z

        #print(f"Received x:{msg.linear.x, self.tool_vel[0]} og y:{msg.linear.y, self.tool_vel[1]}")

        #Anglular 1
        self.theta1 += self.tool_vel[0]
        self.theta1 = np.clip(self.theta1, 0, 180)

        #Linear 2
        self.theta2 = self.tool_vel[2] * 200.0
        self.theta2 = np.clip(self.theta2, -254, 254)

        #Angular 3
        self.theta3 += self.tool_vel[3]
        self.theta3 = np.clip(self.theta3, 0, 180)

        #Angular 4
        self.theta4 += self.tool_vel[1]
        self.theta4 = np.clip(self.theta4, 0, 180)

        if self.debug_log:
            self.get_logger().info(f"Calculated angles: {self.theta1}, {self.theta2}, {self.theta3}, {self.theta4}")

        # Int and correction for servo control
        servo_theta1 = int(self.theta1)
        servo_theta2 = int(self.theta2)
        servo_theta3 = int(self.theta3)
        servo_theta4 = int(self.theta4)

        # Array the solution
        servo_motor_pos = np.array([servo_theta1, servo_theta2, 0, servo_theta3, 0, servo_theta4])

        # Send servo positions
        try:
            #self.servo_controller.send_servo_values(self.arm_control.motor_pos)
            self.servo_controller.send_servo_values(servo_motor_pos)
            if self.debug_log:
                self.get_logger().info(f"Sent servo positions: {servo_motor_pos}")
        except Exception as e:
            self.get_logger().error(f"Error sending servo positions: {e}")


def main(args=None):
    rclpy.init(args=args)
    servo_node = ServoNode()
    try:
        rclpy.spin(servo_node)
    except KeyboardInterrupt:
        pass
    finally:
        servo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()