# IceCube
# Main node for Etri Mjonir Arm control
#
# UiA Grimstad
# 25/9 - 24
#

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
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value

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
        self.arm_inv_control = RoboticArmIK(0.065, 0.35, 0.304)
        self.theta1 = 90
        self.theta2 = 90
        self.theta3 = 90

        # Subscribe to the Twist topic
        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.twist_callback,
            10
        )

        self.get_logger().info(f"Servo node initialized. Listening to {self.cmd_vel_topic}")

    def twist_callback(self, msg):
        # Process the Twist message and compute servo positions
        # Map velocities to servo positions
        servo_positions = np.array([90,90,90,90,90,90])  # Default positions

        # Map linear velocities to servos 1-3
        servo_positions[0] = msg.linear.x
        servo_positions[1] = msg.linear.y
        servo_positions[2] = msg.linear.z

        # Map angular velocities to servos 4-6
        servo_positions[3] = msg.angular.x
        servo_positions[4] = msg.angular.y
        servo_positions[5] = msg.angular.z

        #self.arm_control.calculate_joint_vel_array(servo_positions)
        self.arm_control.integrate_tool_pos(servo_positions)

        print(f"Servo positions: {self.arm_control.tool_pos}")
        solution = self.arm_inv_control.inverse_kinematics(self.arm_control.tool_pos[0], self.arm_control.tool_pos[1], self.arm_control.tool_pos[2])

        if solution != None:
            self.theta1 = int(solution[0][0])
            self.theta2 = int(solution[0][1])
            self.theta3 = 180-int(solution[0][2])
            #print(f"1: {self.theta1}, 2: {theta2}, 3: {theta3}")
        else:
            print("No valid solution")


        servo_motor_pos = np.array([self.theta1, self.theta2, self.theta3, 90, 90, 90])
        print("Servomotor pos: {servo_motor_pos}")
        # Send servo positions
        try:
            #self.servo_controller.send_servo_values(self.arm_control.motor_pos)
            self.servo_controller.send_servo_values(servo_motor_pos)
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