'''
Martin Økter
# 12.09.2024

This is library are meant to implement the control mechanisms
for the arm used on the IEEE Eitri as an easier alternative - Mjolner Robot
'''

import numpy as np
from scipy.optimize import fsolve


class RoboticArmIK:
    def __init__(self, L1, L2, L3, joint_limits=None):
        """
        Initialize the robotic arm with given link lengths.

        Parameters:
        L1, L2, L3 : float
            Lengths of the robot's links.
        joint_limits : dict, optional
            Dictionary specifying the joint limits in degrees.
            Example:
            joint_limits = {
                'theta1': (-180, 180),
                'theta2': (-90, 90),
                'theta3': (-180, 180),
            }
        """
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.joint_limits = joint_limits

    def inverse_kinematics(self, x, y, z):
        """
        Compute the inverse kinematics for the given end-effector position.

        Parameters:
        x, y, z : float
            Desired end-effector coordinates.

        Returns:
        solutions : list of tuples
            List of possible solutions (theta1, theta2, theta3) in degrees.
        """
        # Calculate theta1
        theta1 = np.arctan2(y, x)
        r = np.hypot(x, y)

        # Define the equations to solve
        def equations(vars):
            theta2, theta3 = vars
            eq1 = z - (self.L1 + self.L2 * np.cos(theta2) + self.L3 * np.cos(theta3 - theta2))
            eq2 = r - (self.L2 * np.sin(theta2) + self.L3 * np.sin(theta3 - theta2))
            return [eq1, eq2]

        # Initial guesses (in radians)
        initial_guesses = [
            [np.deg2rad(50), np.deg2rad(-140)],
            [np.deg2rad(-45), np.deg2rad(-45)],
            [np.deg2rad(135), np.deg2rad(45)],
            [np.deg2rad(-135), np.deg2rad(-45)],
        ]

        solutions = []
        for initial_guess in initial_guesses:
            try:
                solution, infodict, ier, mesg = fsolve(equations, initial_guess, full_output=True)
                if ier != 1:
                    continue  # Skip if the solution did not converge
                theta2_sol, theta3_sol = solution
                theta1_deg = np.degrees(theta1)
                theta2_deg = np.degrees(theta2_sol)
                theta3_deg = np.degrees(theta3_sol)

                # Normalize angles between -180 and 180 degrees
                theta1_deg = ((theta1_deg + 180) % 360) - 180
                theta2_deg = ((theta2_deg + 180) % 360) - 180
                theta3_deg = ((theta3_deg + 180) % 360) - 180

                # Check joint limits if provided
                if self.joint_limits:
                    if not (self.joint_limits['theta1'][0] <= theta1_deg <= self.joint_limits['theta1'][1]):
                        continue
                    if not (self.joint_limits['theta2'][0] <= theta2_deg <= self.joint_limits['theta2'][1]):
                        continue
                    if not (self.joint_limits['theta3'][0] <= theta3_deg <= self.joint_limits['theta3'][1]):
                        continue

                # Avoid duplicates
                solution_tuple = (theta1_deg, theta2_deg, theta3_deg)
                if solution_tuple not in solutions:
                    solutions.append(solution_tuple)
            except Exception:
                continue  # Skip if an error occurs

        if not solutions:
            raise ValueError("No valid solutions found for the given position.")

        return solutions

    def forward_kinematics(self, theta1_deg, theta2_deg, theta3_deg):
        """
        Compute the forward kinematics for the given joint angles.

        Parameters:
        theta1_deg, theta2_deg, theta3_deg : float
            Joint angles in degrees.

        Returns:
        x, y, z : float
            End-effector coordinates.
        """
        theta1_rad = np.deg2rad(theta1_deg)
        theta2_rad = np.deg2rad(theta2_deg)
        theta3_rad = np.deg2rad(theta3_deg)

        D = self.L2 * np.sin(theta2_rad) + self.L3 * np.sin(theta3_rad - theta2_rad)
        x = np.cos(theta1_rad) * D
        y = np.sin(theta1_rad) * D
        z = self.L1 + self.L2 * np.cos(theta2_rad) + self.L3 * np.cos(theta3_rad - theta2_rad)
        return x, y, z
