"""
This file records a physcial toolpath and then plays it back to see what resolution we can get
the motors working at.
"""

from motor_controller import MotorController, bits_to_degrees, degrees_to_bits, getch
from forward_kinematics import forward_kinematics
import time
import numpy as np
from constants import ANGLE_OFFSET, ANGLE_SCALING, MOTOR_IDS


COM_PORT = 'COM5'
RUNTIME_S = 5 # seconds
STEP_TIME_S = 0.05 # seconds


ESC_CH = 0x1b


# initialize dynamixel motors
controller = MotorController(COM_PORT, MOTOR_IDS)
controller.connect_dynamixel()

print("Press any key to start recording")
getch()

# record the toolpath
recording_bits = np.array([0, 0, 0, 0], dtype=int)
recording_step_time = [0]
start_time = time.time()
last_recording_time = start_time
while time.time() - start_time < RUNTIME_S:
    # get the current motor positions
    positions_bits = controller.get_motor_positions()
    recording_bits = np.vstack((recording_bits, positions_bits))
    recording_time = time.time()
    recording_step_time.append(recording_time - last_recording_time) 
    last_recording_time = recording_time


print("Press any key to start playback")
getch()
controller.write_motor_positions(recording_bits[1])
controller.enable_all_torque()
time.sleep(1)
for i in range(1, recording_bits.shape[0]):
    controller.write_motor_positions(recording_bits[i])
    sleep_time = recording_step_time[i] - (time.time() - last_recording_time)
    if sleep_time > 0:
        time.sleep(sleep_time)
    last_recording_time = time.time()

controller.disconnect()