'''This file contains the constants used in the kinematic model of the robowriter robot'''

L0 = 100            # mm  [BASE TO AXIS 2 LENGTH] (need to verify)
L2 = 140            # mm
L3 = 140            # mm
L4 = 40.389         # mm
L5 = 25             # mm  [PEN EXTENTION LENGTH]

THETA_5 =  50.25      # degrees


# Joint angle limits as defined by our kinematic model without offsets.
THETA_1_MIN, THETA_1_MAX = -55, 55      # degrees
THETA_2_MIN, THETA_2_MAX = -115, 0      # degrees
THETA_3_MIN, THETA_3_MAX = -108, 0      # degrees
THETA_4_MIN, THETA_4_MAX = -103, 103    # degrees

PEN_LIFT_MM = 10        # mm  [PEN LIFT HEIGHT]
MAX_STEP_MM = 1       # mm  [MAXIMUM DISTANCE BETWEEN POINTS IN THE TOOLPATH]

ROBOT_HOME_POSITION = [200, 0, 50] # [x, y, z] in mm in base coordinate frame.
