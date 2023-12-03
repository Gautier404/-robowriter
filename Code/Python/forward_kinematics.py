"""
This file describes the forward kinematic model of the robot and is adapted from Sean's Matlab code.

You can find a description of the robot's geometry in the DH_and_frames.jpg file.
"""
import numpy as np
import pandas as pd
from kinematic_constants import *


def transformation_matrix(alpha, a, d, theta):
    """
    Calculates the transformation matrix for a given set of DH parameters.

    parameters:
        alpha: angle between z_i-1 and z_i measured about x_i-1 in degrees
        a: distance between x_i-1 and x_i measured along z_i-1 in mm
        d: distance between z_i-1 and z_i measured along x_i in mm
        theta: angle between x_i-1 and x_i measured about z_i-1 in degrees
    """
    t = np.array(
        [
            [np.cos(np.radians(theta)), -np.sin(np.radians(theta)), 0, a],
            [
                np.sin(np.radians(theta)) * np.cos(np.radians(alpha)),
                np.cos(np.radians(theta)) * np.cos(np.radians(alpha)),
                -np.sin(np.radians(alpha)),
                -np.sin(np.radians(alpha)) * d,
            ],
            [
                np.sin(np.radians(theta)) * np.sin(np.radians(alpha)),
                np.cos(np.radians(theta)) * np.sin(np.radians(alpha)),
                np.cos(np.radians(alpha)),
                np.cos(np.radians(alpha)) * d,
            ],
            [0, 0, 0, 1],
        ]
    )

    return t


def forward_kinematics(theta1: float, theta2: float, theta3: float, theta4: float):
    """
    Calculates the transformation matricies for each join angle

    parameters:
        theta1: angle of joint 1 in degrees
        theta2: angle of joint 2 in degrees
        theta3: angle of joint 3 in degrees
        theta4: angle of joint 4 in degrees

    returns:
        t: a list of transformation matricies that maps the base frame to each joint.
    """
    t = [None] * 6
    tf_1 = transformation_matrix(0, 0, 0, theta1)
    t[0] = tf_1
    tf_2 = transformation_matrix(90, 0, 0, 90 + theta2)
    t[1] = t[0] @ tf_2
    tf_3 = transformation_matrix(0, L2, 0, theta3)
    t[2] = t[1] @ tf_3
    tf_4 = transformation_matrix(0, L3, 0, theta4)
    t[3] = t[2] @ tf_4
    tf_5 = transformation_matrix(0, L4, 0, THETA_5)
    t[4] = t[3] @ tf_5
    tf_6 = transformation_matrix(0, L5, 0, 0)
    t[5] = t[4] @ tf_6
    return t


def generate_link_coordinates(
    theta1: float, theta2: float, theta3: float, theta4: float
):
    """
    Generates an dict of the cartesian coordinates of each joint in the robot arm.

    parameters:
        theta1: angle of joint 1 in degrees
        theta2: angle of joint 2 in degrees
        theta3: angle of joint 3 in degrees
        theta4: angle of joint 4 in degrees
    """
    # pylint: disable=E1136  # pylint/issues/3139

    transformation_matrices = forward_kinematics(theta1, theta2, theta3, theta4)

    # Joint positions
    x_coords = [
        0,
        transformation_matrices[0][0, 3],
        transformation_matrices[1][0, 3],
        transformation_matrices[2][0, 3],
        transformation_matrices[3][0, 3],
        transformation_matrices[4][0, 3],
        transformation_matrices[5][0, 3],
    ]
    y_coords = [
        0,
        transformation_matrices[0][1, 3],
        transformation_matrices[1][1, 3],
        transformation_matrices[2][1, 3],
        transformation_matrices[3][1, 3],
        transformation_matrices[4][1, 3],
        transformation_matrices[5][1, 3],
    ]
    z_coords = [
        0,
        transformation_matrices[0][2, 3],
        transformation_matrices[1][2, 3],
        transformation_matrices[2][2, 3],
        transformation_matrices[3][2, 3],
        transformation_matrices[4][2, 3],
        transformation_matrices[5][2, 3],
    ]

    return {"x": x_coords, "y": y_coords, "z": z_coords}


if __name__ == "__main__":
    # Example usage
    # pylint: disable=E1136  # pylint/issues/3139

    T1 = 45
    T2 = 30
    T3 = -60
    T4 = 90

    resulting_transformation = forward_kinematics(T1, T2, T3, T4)
    print("Resulting Transformation Matrix:")
    for m in resulting_transformation:
        print(m)

    print(resulting_transformation[4][0, 3])
