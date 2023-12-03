import numpy as np
from kinematic_constants import *
from static_model import plot_3d_robot_arm

def generate_link_angles(pen_position: np.array):
    """
    Outputs an array of link angles of the robot given a target pen tip position.
    
    parameters:
        pen_position: a 1D array of the x, y, z coordinates of the pen tip in mm 
            in the base coordinate frame.
    """
    x, y, z = pen_position
    
    # Calculate theta1
    theta1 = np.degrees(np.arctan2(y, x))

    # determine location of x5, y5, z5
    # These calculations assume the pen is normal to a flat surface on the same plane as the base of the robot
    x5 = x
    y5 = y
    z5 = z + L5

    # determine location of x4, y4, z4
    x4 = x5 + np.cos(np.radians(theta1))*np.sin(np.radians(-THETA_5))*L4
    y4 = y5 + np.sin(np.radians(theta1))*np.sin(np.radians(-THETA_5))*L4
    z4 = z5 + np.cos(np.radians(-THETA_5))*L4

    # check to see if our point is too far away for the robot to reach TODO: make this acurate
    if np.sqrt(x4**2 + y4**2 + (z4 - L0)**2) > L2 + L3:
        print("Target point is too far away for the robot to reach.")
        return None

    # Calculate theta 3
    # This calculation is derived geometrically from the 3R 3D robot arm inverse kinematic example in the textbook
    r2 = np.sqrt(x4**2 + y4**2 + (z4 - L0)**2)
    c3 = (r2**2 - L2**2 - L3**2) / (2 * L2 * L3)
    s3 = np.sqrt(1 - c3**2)
    theta3 = np.degrees(np.arctan2(-s3, c3)) #this assumes that the triangle formed by link 2 and 3 has an obtuse side pointed up

    # Calculate theta 2
    alpha = np.degrees(np.arctan2(z4 - L0, np.sqrt(x4**2 + y4**2)))
    cbeta = (L2**2 + r2**2 - L3**2) / (2 * L2 * r2)
    sbeta = np.sqrt(1 - cbeta**2)
    beta = np.degrees(np.arctan2(sbeta, cbeta)) #this assumes that the triangle formed by link 2 and 3 has an obtuse side pointed up
    theta2 = 90 - alpha - beta

    # Calculate theta 4
    theta4 = -(90 + theta2 + theta3 + THETA_5)

    return [theta1, theta2, theta3, theta4]

if __name__ == "__main__":
    pen_position = np.array([100, 100, 0])
    link_angles = generate_link_angles(pen_position)
    print(link_angles)

    plot_3d_robot_arm(link_angles[0], link_angles[1], link_angles[2], link_angles[3])
    