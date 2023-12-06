"""
This file is intended to be used to figure out the correct motor offsets to
get the actual motors positions to line up with our modeled ones
"""

from motor_controller import MotorController, bits_to_degrees, degrees_to_bits, getch
from forward_kinematics import forward_kinematics
import time
import numpy as np
from constants import ANGLE_OFFSET, ANGLE_SCALING, MOTOR_IDS


COM_PORT = 'COM5'
RUNTIME_S = 100 # seconds
STEP_TIME_S = 0.1 # seconds


ESC_CH = 0x1b


# initialize dynamixel motors
controller = MotorController(COM_PORT, MOTOR_IDS)
controller.connect_dynamixel()

for i in range(0, int(RUNTIME_S/STEP_TIME_S)):
    # get the current motor positions
    positions_bits = controller.get_motor_positions()
    positions_degrees_physical = bits_to_degrees(positions_bits)
    positions_degrees_theoretical = ANGLE_SCALING * (positions_degrees_physical - ANGLE_OFFSET)
    end_deffector_coords = forward_kinematics(positions_degrees_theoretical[0], positions_degrees_theoretical[1], positions_degrees_theoretical[2], positions_degrees_theoretical[3])[5][0:3, 3]
    print("Current motor positions: ", positions_degrees_theoretical, "end deffector coords: ", end_deffector_coords)
    time.sleep(STEP_TIME_S)

controller.disconnect()

