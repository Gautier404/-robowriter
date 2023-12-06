"""This file uses the motor controller class to release the torque on all the motors"""
from motor_controller import MotorController
from constants import MOTOR_IDS

COM_PORT = 'COM3'

controller = MotorController(COM_PORT, MOTOR_IDS)
controller.connect_dynamixel()
controller.disable_all_torque()
controller.disconnect()
