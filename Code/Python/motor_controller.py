from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import os
import time
import numpy as np

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
# Control table address
TORQUE_ENABLE_ADDRESS      = 64               # Control table address is different in Dynamixel model
GOAL_POSITION_ADDRESS      = 116
PRESENT_POSITION_ADDRESS   = 132

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM5'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0                 # Minimum position limit
DXL_MAXIMUM_POSITION_VALUE  = 4095              # Maximum position limit
DXL_MOVING_STATUS_THRESHOLD = 1                 # Dynamixel moving status threshold

BITS_TO_DEGREES = 360 / 4096
DEGREES_TO_BITS = 4096 / 360

class MotorController:
    """Class for simplifying the control of multiple dynamixel motors"""
    def __init__(self, port: str = 'COM5', motor_ids: np.array = np.array([5])) -> None:
        """
        Initialize the motor controller
        parameters:
            port: the port to connect to the dynamixel motors
            motor_ids: the ids of the dynamixel motors to control in a numpy array
        """
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.motor_ids = motor_ids
    
    def connect_dynamixel(self) -> None:
        """Connect to the dynamixel motors"""
        # Initialize port_handler instance
        if self.port_handler.openPort():
            print("Succeeded to open the dynamixel port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()
        
        # Set port baudrate
        if self.port_handler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
    
    def disconnect(self)-> None:
        "Disconnect from the dynamixel motors"
        self.disable_all_torque()
        self.port_handler.closePort()
        
    def enable_motor_torque(self, motor_id: int):
        "Enable the torque for the given dynamixel motor"
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, TORQUE_ENABLE_ADDRESS, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")
    
    def enable_all_torque(self):
        "Enable the torque for all the dynamixel motors"
        for motor_id in self.motor_ids:
            self.enable_motor_torque(motor_id)
    
    def disable_motor_torque(self, motor_id: int):
        "Disable the torque for the given dynamixel motor"
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, TORQUE_ENABLE_ADDRESS, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))
    
    def disable_all_torque(self):
        "Disable the torque for all the dynamixel motors"
        for motor_id in self.motor_ids:
            self.disable_motor_torque(motor_id)
    
    def get_motor_position(self, motor_id: int)-> int:
        "Read the current position of the given dynamixel motor in bits and return it"
        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, PRESENT_POSITION_ADDRESS)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))
        
        #print("[ID:%03d] PresPos:%03d" % (motor_id, dxl_present_position))
        return dxl_present_position
    
    def get_motor_positions(self)-> np.array:
        "Read the current position of all the dynamixel motors and return them in a numpy array"
        positions = np.array([], dtype=int)
        for motor_id in self.motor_ids:
            positions = np.append(positions, self.get_motor_position(motor_id))
        return positions
    
    def write_motor_position(self, motor_id: int, position: int):
        "Write the given position to the given dynamixel motor"
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, GOAL_POSITION_ADDRESS, position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))
    
    def write_motor_positions(self, positions: np.array):
        """
        Write the given positions to the dynamixel motors
        parameters:
            positions: a numpy array of the positions to write to the motors in the same order 
                as the motor ids were given in the constructor
        """
        # check that positions and motor id's are of same dimension
        if len(positions) != len(self.motor_ids):
            self.disconnect()
            raise ValueError("positions and motor ids are not of same dimension")
        
        # write each position to the corresponding motor 
        for i, position in enumerate(positions):
            self.write_motor_position(self.motor_ids[i], position)

def bits_to_degrees(bits: np.array)-> np.array:
    "Convert the given bits to degrees"
    return bits * BITS_TO_DEGREES

def degrees_to_bits(degrees: np.array)-> np.array:
    "Convert the given degrees to bits"
    return (degrees * DEGREES_TO_BITS).astype(int)


if __name__ == '__main__':
    # tested with motor id 5 and 6
    motor_ids = np.array([5, 6])
    motor_id = 5
    motors = MotorController('COM5', motor_ids)
    motors.connect_dynamixel()

    # for i in range(0,100):
    #     motors.get_motor_positions()
    #     time.sleep(0.1)
    # motors.disconnect()


    motors.enable_motor_torque(motor_id)
    motors.get_motor_position(motor_id)
    motors.write_motor_position(motor_id, 2048)
    time.sleep(1)
    motors.write_motor_position(motor_id, 0)
    motors.enable_motor_torque(6)
    print("reading motor 6")
    motors.get_motor_position(6)
    motors.write_motor_position(6, 2048)
    time.sleep(1)
    motors.write_motor_position(6, 0)
    time.sleep(1)
    print(motors.get_motor_positions())
    motors.write_motor_positions(np.array([2048, 2048]))
    time.sleep(1)
    motors.write_motor_positions(np.array([0, 0]))
    motors.get_motor_positions()
    time.sleep(1)
    motors.disconnect()

    print(degrees_to_bits(np.array([0, 15.5, 90, 270, 360])))
    print(bits_to_degrees(np.array([0, 2048, 4095])))