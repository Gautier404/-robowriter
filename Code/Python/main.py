from SVG_to_coords import *
from coords_to_toolpath import *
from generate_toolpath import *
from fit_path import *
from animate_arm import *
import plotly.graph_objects as go
from constants import *
from motor_controller import MotorController, bits_to_degrees, degrees_to_bits, getch
import time
from forward_kinematics import *

# Get svg to use.
current_working_directory = os.getcwd()
file_name = input("Input svg name you wish to use. It must be in the svg directory\n")
svg_path = "\\Code\\Python\\svgs\\"
cwd_split = current_working_directory.split('\\')
if cwd_split[len(cwd_split)-1] == "robowriter":
    file_path = current_working_directory + svg_path + file_name + '.svg'
else:
    file_path = current_working_directory + '\\robowriter' + svg_path + file_name + '.svg'

# Convert svg to 2d toolpath.
print("Converting svg to 2d toolpath...")
coords = parse_svg(file_path)
toolpath = read_path(coords)
# for i in range(0, len(toolpath)):
#     for j in range(0, len(toolpath[i])):
#         toolpath[i][j][0] = float(toolpath[i][j][0])
#         toolpath[i][j][1] = float(toolpath[i][j][1])

# Scale toolpath to fit in bounds.
print("Scaling toolpath...")
fitted_toolpath = fit_path(toolpath, DRAWING_BOUNDS)
consolidated_toolpath = np.concatenate(toolpath)
consolidated_fitted_toolpath = np.concatenate(fitted_toolpath)

print("Displaying scaled toolpath...")
# Create figure
scaled_toolpaths = go.Figure()

# Plot original toolpath
scaled_toolpaths.add_trace(go.Scatter(x=consolidated_toolpath[:,0], y=consolidated_toolpath[:,1], mode='lines', name='Original Toolpath'))

# Plot bounds
scaled_toolpaths.add_trace(go.Scatter(x=DRAWING_BOUNDS[:,0], y=DRAWING_BOUNDS[:,1], mode='markers', name='Bounds'))

# Plot scaled toolpath
scaled_toolpaths.add_trace(go.Scatter(x=consolidated_fitted_toolpath[:,0], y=consolidated_fitted_toolpath[:,1], mode='lines', name='Scaled Toolpath'))

# Set layout
scaled_toolpaths.update_layout(title='Scaled Toolpath Plot',
                    showlegend=True)

scaled_toolpaths.show()

# Generate 3d toolpath.
print("Generating 3d toolpath...")
cartesian_toolpath = generate_cartesian_toolpath(fitted_toolpath)

# Plot 3d toolpath.
print("Displaying 3d toolpath...")
toolpath_3d_fig = go.Figure()
toolpath_3d_fig.add_trace(go.Scatter3d(x=cartesian_toolpath[:,0], y=cartesian_toolpath[:,1], z=cartesian_toolpath[:,2], mode='markers', name = 'Interpolated Toolpath', marker=dict(color="blue", size=2)))
toolpath_3d_fig.show()

# Generate angular toolpath.
print("Generating angular toolpath...")
angular_toolpath_model = generate_angular_toolpath(cartesian_toolpath)

# ask whether or not to play animation:
print("Do you want to play the animation? (y)")
if input() == "y":
    print("Generating animation...")
    animate_arm(angular_toolpath_model, cartesian_toolpath)
else:
    print("Skipping animation...")


# Apply transformations to convert model angles into physical bit commands.
print("Converting model angles into bit commands...")
angular_toolpath_physical = angular_toolpath_model*ANGLE_SCALING + ANGLE_OFFSET
bit_commands = degrees_to_bits(angular_toolpath_physical)


# Initialize dynamixel motors
print("Press any key to continue to path execution or ESC to cancel")
if ord(getch()) == ESC_CH:
    exit()
print("Initializing motors...")
COM_PORT = 'COM5'
controller = MotorController(COM_PORT, MOTOR_IDS)
controller.connect_dynamixel()
controller.write_motor_positions(HOME_POSITION_BITS)
controller.enable_all_torque()
time.sleep(1)




# Run profile
print("Running profile...")
SPEED = 3
positions_bits = controller.get_motor_positions()
positions_degrees_physical = bits_to_degrees(positions_bits)
positions_degrees_theoretical = ANGLE_SCALING * (positions_degrees_physical - ANGLE_OFFSET)
end_deffector_coords = forward_kinematics(positions_degrees_theoretical[0], positions_degrees_theoretical[1], positions_degrees_theoretical[2], positions_degrees_theoretical[3])[5][0:3, 3]
output_toolpath = end_deffector_coords
output_angles = positions_degrees_theoretical
for i, commands in enumerate(bit_commands):
    if i % SPEED == 0:
        controller.write_motor_positions(commands)
        positions_bits = controller.get_motor_positions()
        positions_degrees_physical = bits_to_degrees(positions_bits)
        positions_degrees_theoretical = ANGLE_SCALING * (positions_degrees_physical - ANGLE_OFFSET)
        end_deffector_coord = forward_kinematics(positions_degrees_theoretical[0], positions_degrees_theoretical[1], positions_degrees_theoretical[2], positions_degrees_theoretical[3])[5][0:3, 3]
        output_toolpath = np.vstack((output_toolpath, end_deffector_coord))
        output_angles = np.vstack((output_angles, positions_degrees_theoretical))

# Disconnect motors
print("Do you want to disconnect the motors? (y)")
if input() == "y":
    print("Disconnecting motors...")
    controller.disconnect()


<<<<<<< HEAD
# Save toolpath to file
print("Do you want to save the toolpath to a file? (y)")
if input() == "y":
    print("Saving toolpath...")
    np.savetxt("toolpath.txt", bit_commands, fmt='%d')
    print("Toolpath saved to toolpath.txt")
=======
#Enter filename to save
answer = input("Do you want to save the toolpath to a file? (y)")
if answer == "y":
    filename = input("Enter file name")
    print("Saving toolpath... ")
    np.savetxt(filename+"_input_toolpath.txt", cartesian_toolpath, fmt = '%d')
    np.savetxt(file_name+"_output_toolpath.txt", output_toolpath, fmt = '%d')
    print("files saved")


post_review_3d = go.Figure()

# Plot original toolpath
post_review_3d.add_trace(go.Scatter3d(x=cartesian_toolpath[:,0], y=cartesian_toolpath[:,1], z=cartesian_toolpath[:,2], mode='lines', name='Input Toolpath'))

# Plot scaled toolpath
post_review_3d.add_trace(go.Scatter3d(x=output_toolpath[:,0], y=output_toolpath[:,1], z = output_toolpath[:,2] ,mode='lines', name='Recorded Toolpath'))

# Set layout
post_review_3d.update_layout(title='Post Review',
                    showlegend=True)

post_review_3d.show()

# Add figure of angle commands
post_review_angles = go.Figure()

x_output = np.linspace(0, output_angles.shape[0], output_angles.shape[0])*SPEED
x_input = np.linspace(0, angular_toolpath_model.shape[0], angular_toolpath_model.shape[0])
post_review_angles.add_trace(go.Scatter(x = x_output,  y = output_angles[:,0], name="Theta_1_out"))
post_review_angles.add_trace(go.Scatter(x = x_output,  y = output_angles[:,1], name="Theta_2_out"))
post_review_angles.add_trace(go.Scatter(x = x_output,  y = output_angles[:,2], name="Theta_3_out"))
post_review_angles.add_trace(go.Scatter(x = x_output,  y = output_angles[:,3], name="Theta_4_out"))
post_review_angles.add_trace(go.Scatter(x = x_input,  y = angular_toolpath_model[:,0], name="Theta_1_in"))
post_review_angles.add_trace(go.Scatter(x = x_input,  y = angular_toolpath_model[:,1], name="Theta_2_in"))
post_review_angles.add_trace(go.Scatter(x = x_input,  y = angular_toolpath_model[:,2], name="Theta_3_in"))
post_review_angles.add_trace(go.Scatter(x = x_input,  y = angular_toolpath_model[:,3], name="Theta_4_in"))

post_review_angles.show()
>>>>>>> 47c44772fdddfeeab4b59c9b3d40b59153be3da2
