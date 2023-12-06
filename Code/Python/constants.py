'''This file contains the constants used in the kinematic model of the robowriter robot'''
import numpy as np
from motor_controller import bits_to_degrees, degrees_to_bits

L0 = 118.8          # mm  [BASE TO AXIS 2 LENGTH] (need to verify)
L2 = 140            # mm
L3 = 140            # mm
L4 = 39         # mm
L5 = 59          # mm  [PEN EXTENTION LENGTH]

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

PEN_LIFT_MM = 10        # mm  [PEN LIFT HEIGHT]
MAX_STEP_MM = 1       # mm  [MAXIMUM DISTANCE BETWEEN POINTS IN THE TOOLPATH]



ANGLE_OFFSET = np.array([139.57, 179.03, 160.66, 96.59])  # Converts from physical angles to theoretical ones
ANGLE_SCALING = np.array([1, -1, -1, -1])               # Converts from theoretical angles to physical ones
                                                        # theoretical_angle = ANGLE_SCALING * (physical_angle - OFFSET_ANGLES_GUESS)

HOME_POSITION_CARTESIAN = [160, 0, 60] # [x, y, z] in mm in base coordinate frame.
HOME_POSITION_DEGREES_MODEL = [0.0, 16.042820445398846, 123.81428707308204, -10.107107518480888] # A stable home position in degrees as calculated by our model.
HOME_POSITION_BITS = degrees_to_bits(HOME_POSITION_DEGREES_MODEL*ANGLE_SCALING + ANGLE_OFFSET) # A stable home position in bits.

DRAWING_BOUNDS = np.array([[280, 125], [120, -100]]) # The bounds of the drawing area in mm in the base coordinate frame.

MOTOR_IDS = [1, 2, 3, 4]                                # The motor ids for the motors we are using.
ESC_CH = 0x1b                                           # The escape character used to exit the program.
TABLE_HEIGHT_MM = 5                                   #  The height of the table in mm in the base coordinate frame.
