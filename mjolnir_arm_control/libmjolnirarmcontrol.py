'''
Martin Økter
# 12.09.2024

This is library are meant to impliment the control mechanisms
for the arm used on the IEEE Eitri - Mjolner Robot
'''

import numpy as np
import time as t


class MjolnirArmControl:
    def __init__(self):
        self.name = "Hammer Retreiver"

        self.last_time = t.time()

        self.motor_vel = np.array([0, 0, 0, 0, 0, 0])
        self.motor_pos = np.array([90, 90, 90, 90, 90, 90])

        self.inv_jac = np.array([[1, 0, 0, 0, 0, 0],
                                 [0, 1, 0, 0, 0, 0],
                                 [0, 0, 1, 0, 0, 0],
                                 [0, 0, 0, 1, 0, 0],
                                 [0, 0, 0, 0, 1, 0],
                                 [0, 0, 0, 0, 0, 1]])

        self.tool_vel = np.array([0, 0, 0, 0, 0, 0])
        self.tool_pos = np.array([0.30, 0.0, 0.20, 0, 0, 0])

    def calculate_joint_vel(self, x, y, z, ax, ay, az):
        cartesian_vel = np.array([x, y, z, ax, ay, az])
        self.motor_vel = np.matmul(self.inv_jac, cartesian_vel)

    def calculate_joint_vel_array(self, joint_list: list):
        self.motor_vel = np.matmul(self.inv_jac, joint_list)
        self.integrate_motor_vel()

    def integrate_motor_vel(self):
        loop_time = t.time() - self.last_time
        self.last_time = t.time()
        self.motor_pos = self.motor_pos + (self.motor_vel * loop_time)

    def integrate_tool_pos(self, tool_vel_in: np.array):
        loop_time = t.time() - self.last_time
        self.last_time = t.time()
        if loop_time > 0 and loop_time < 0.5:
            self.tool_pos = self.tool_pos + (tool_vel_in * loop_time)

