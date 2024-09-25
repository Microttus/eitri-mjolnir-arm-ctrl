# servo_node_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo_node',
            executable='servo_node',
            name='servo_node',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'baud_rate': 115200},
                {'cmd_vel_topic': '/cmd_vel'},
                {'max_linear_velocity': 1.0},
                {'max_angular_velocity': 1.0},
            ],
        ),
    ])
