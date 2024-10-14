# IceCube
# Main node for Etri Mjonir Arm control
#
# UiA Grimstad
# 25/9 - 24
#

import rclpy
from rcl.node import Node
from geometry_msgs.msg import Twist
from libserialservo import ServoController
from libmjolnirarmcontrol import MjolnirArmControl

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
        servo_positions = [90] * 6  # Default positions

        # Map linear velocities to servos 1-3
        servo_positions[0] = msg.linear.x
        servo_positions[1] = msg.linear.y
        servo_positions[2] = msg.linear.z

        # Map angular velocities to servos 4-6
        servo_positions[3] = msg.angular.x
        servo_positions[4] = msg.angular.y
        servo_positions[5] = msg.angular.z

        self.arm_control.calculate_joint_vel_array(servo_positions)

        # Send servo positions
        try:
            self.servo_controller.send_servo_values(self.arm_control.motor_pos)
            self.get_logger().info(f"Sent servo positions: {self.arm_control.motor_pos}")
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