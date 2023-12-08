"""Create a toolpath that moves in a rectangle defined by the drawing bounds."""
from coords_to_toolpath import *
from generate_toolpath import *
from fit_path import *
from animate_arm import *
import plotly.graph_objects as go
from constants import *
from motor_controller import MotorController, bits_to_degrees, degrees_to_bits, getch
import time
from forward_kinematics import *

# define a rectanglular toolpath that moves in a rectangle defined by the drawing bounds
def create_rectangular_toolpath(bounds):

    x_bounds = bounds[:,0]
    y_bounds = bounds[:,1]

    # define the toolpath
    max_x = max(x_bounds)
    min_x = min(x_bounds)
    max_y = max(y_bounds)
    min_y = min(y_bounds)

    toolpath = np.array([[[min_x, min_y],[min_x, max_y],[max_x, max_y],[max_x, min_y],[min_x, min_y]]])
    return toolpath

toolpath = create_rectangular_toolpath(DRAWING_BOUNDS)
fitted_toolpath = fit_path(toolpath, DRAWING_BOUNDS)
cartesian_toolpath = generate_cartesian_toolpath(fitted_toolpath)
angular_toolpath_model = generate_angular_toolpath(cartesian_toolpath)
angular_toolpath_physical = ANGLE_SCALING * (angular_toolpath_model) + ANGLE_OFFSET
bit_commands = degrees_to_bits(angular_toolpath_physical)

print("Press any key to continue to path execution or ESC to cancel")
if ord(getch()) == ESC_CH:
    exit()
print("Initializing motors...")
COM_PORT = 'COM5'
controller = MotorController(COM_PORT, MOTOR_IDS, GAINS)
controller.connect_dynamixel()
controller.enable_all_torque()
time.sleep(1)

# Run profile
print("Running profile...")
SPEED = 1

for i, commands in enumerate(bit_commands):
    if i % SPEED == 0:
        controller.write_motor_positions(commands)
        positions_bits = controller.get_motor_positions()
        positions_degrees_physical = bits_to_degrees(positions_bits)
        positions_degrees_theoretical = ANGLE_SCALING * (positions_degrees_physical - ANGLE_OFFSET)
        print("Current motor positions: ", positions_degrees_physical)

controller.disconnect()