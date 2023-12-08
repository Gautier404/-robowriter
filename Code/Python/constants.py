'''This file contains the constants used in the kinematic model of the robowriter robot'''
import numpy as np

L0 = 118.8          # mm  [BASE TO AXIS 2 LENGTH] (need to verify)
L2 = 140            # mm
L3 = 140            # mm
L4 = 39         # mm
L5 = 63          # mm  [PEN EXTENTION LENGTH]

THETA_5 =  50.25      # degrees


# Joint angle limits as defined by our kinematic model without offsets.
THETA_1_MIN = -60
THETA_1_MAX =  60

THETA_2_MIN = -127
THETA_2_MAX =  135   
   
THETA_3_MIN = -110
THETA_3_MAX =  150 

THETA_4_MIN = -69    
THETA_4_MAX =  99

PEN_LIFT_MM = 5        # mm  [PEN LIFT HEIGHT]
MAX_STEP_MM = 1       # mm  [MAXIMUM DISTANCE BETWEEN POINTS IN THE TOOLPATH]



ANGLE_OFFSET = np.array([139.57, 179.03, 160.66, 96.59])  # Converts from physical angles to theoretical ones
ANGLE_SCALING = np.array([1, -1, -1, -1])               # Converts from theoretical angles to physical ones
                                                        # theoretical_angle = ANGLE_SCALING * (physical_angle - OFFSET_ANGLES_GUESS)

BITS_TO_DEGREES = 360 / 4096
DEGREES_TO_BITS = 4096 / 360

HOME_POSITION_CARTESIAN = [115, 0, 54] # [x, y, z] in mm in base coordinate frame.
HOME_POSITION_DEGREES_MODEL = [0.0, 1.9862793011776887, 143.08261592788622, -15.318895229063912] # A stable home position in degrees as calculated by our model.
HOME_POSITION_BITS = ((HOME_POSITION_DEGREES_MODEL*ANGLE_SCALING + ANGLE_OFFSET)*DEGREES_TO_BITS).astype(int) # A stable home position in bits.

# [300, 190], [130.29, -190] <-  these are pretty good max bounds
# [[300, 130], [130.29, -130]] <- these are the bounds we used for Drew's notebook paper
# [250, 130], [130.29, -130] <- safe bounds
DRAWING_BOUNDS = np.array([[270, 130], [130.29, -130]]) # The bounds of the drawing area in mm in the base coordinate frame.

MOTOR_IDS = [1, 2, 3, 4]                                # The motor ids for the motors we are using.
ESC_CH = 0x1b                                           # The escape character used to exit the program.
TABLE_HEIGHT_MM = -2                                 #  The height of the table in mm in the base coordinate frame.

POINTS_IN_CURVE = 100

GAINS = [
    {
        'P': int(1400),
        'I': int(1500),
        'D': int(0),
    },
    {
        'P': int(1800),
        'I': int(1800),
        'D': int(1000),
    },
    {
        'P': int(850),
        'I': int(1000),
        'D': int(0),
    },
    {
        'P': int(2000),
        'I': int(1500),
        'D': int(500),
    }
]
