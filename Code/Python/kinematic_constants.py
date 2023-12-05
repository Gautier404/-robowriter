'''This file contains the constants used in the kinematic model of the robowriter robot'''
import numpy as np

L0 = 118.8          # mm  [BASE TO AXIS 2 LENGTH] (need to verify)
L2 = 140            # mm
L3 = 140            # mm
L4 = 40.389         # mm
L5 = 35             # mm  [PEN EXTENTION LENGTH]

THETA_5 =  50.25      # degrees


# Joint angle limits as defined by our kinematic model without offsets.
THETA_1_MIN, THETA_1_MAX = -55, 55      # degrees
THETA_2_MIN, THETA_2_MAX = -115, 0      # degrees
THETA_3_MIN, THETA_3_MAX = -108, 0      # degrees
THETA_4_MIN, THETA_4_MAX = -103, 103    # degrees

PEN_LIFT_MM = 10        # mm  [PEN LIFT HEIGHT]
MAX_STEP_MM = 1       # mm  [MAXIMUM DISTANCE BETWEEN POINTS IN THE TOOLPATH]

ROBOT_HOME_POSITION = [200, 0, 50] # [x, y, z] in mm in base coordinate frame.

ANGLE_OFFSET = np.array([135.0, 179.0, 161.0, 100.25])  # Converts from physical angles to theoretical ones
ANGLE_SCALING = np.array([1, -1, -1, -1])               # Converts from theoretical angles to physical ones
                                                        # theoretical_angle = ANGLE_SCALING * (physical_angle - OFFSET_ANGLES_GUESS)

HOME_POSITION = np.array([0, -50, 140, 47])             # A stabile home position in degrees.

MOTOR_IDS = [1, 2, 3, 4]                                # The motor ids for the motors we are using.
