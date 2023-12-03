"""This file uses the forward kinematics model to display our virtual robot arm"""
from forward_kinematics import forward_kinematics
from kinematic_constants import *
import plotly.graph_objects as go
import numpy as np

# def plot_3d_robot_arm(theta1, theta2, theta3, theta4):
#     """
#     takes in the angles of th robot arm and plots its configuration using plotly
#     """
#     # pylint: disable=E1136  # pylint/issues/3139

#     # Calculate forward kinematics

#     transformation_matrices = forward_kinematics(theta1, theta2, theta3, theta4)

#     # Joint positions
#     x_coords = [
#          0,
#          transformation_matrices[0][0, 3], 
#          transformation_matrices[1][0, 3], 
#          transformation_matrices[2][0, 3], 
#          transformation_matrices[3][0, 3], 
#          transformation_matrices[4][0, 3], 
#          transformation_matrices[5][0, 3]]
#     y_coords = [
#          0,
#          transformation_matrices[0][1, 3], 
#          transformation_matrices[1][1, 3], 
#          transformation_matrices[2][1, 3], 
#          transformation_matrices[3][1, 3], 
#          transformation_matrices[4][1, 3], 
#          transformation_matrices[5][1, 3]]
#     z_coords = [
#          0,
#          transformation_matrices[0][2, 3], 
#          transformation_matrices[1][2, 3], 
#          transformation_matrices[2][2, 3], 
#          transformation_matrices[3][2, 3], 
#          transformation_matrices[4][2, 3], 
#          transformation_matrices[5][2, 3]]
    

#     # Create figure
#     fig = go.Figure()

#     # Plot links
#     fig.add_trace(go.Scatter3d(x=x_coords, y=y_coords, z=z_coords,
#                                mode='lines+markers', name='Robot Arm'))

#     # Plot joints
#     fig.add_trace(go.Scatter3d(x=x_coords, y=y_coords, z=z_coords,
#                                mode='markers', marker=dict(color='red'), name='Joints'))

#     # Set layout
#     fig.update_layout(title='3D Robot Arm Configuration',
#                       scene=dict(aspectmode='data'),
#                       showlegend=True)

#     # Show plot
#     fig.show()

def plot_3d_robot_arm(theta1, theta2, theta3, theta4):
    """
    Takes in the angles of the robot arm and plots its configuration using plotly.
    """
    # Calculate forward kinematics
    transformation_matrices = forward_kinematics(theta1, theta2, theta3, theta4)

    # Joint positions
    x_coords = [tm[0, 3] for tm in transformation_matrices]
    y_coords = [tm[1, 3] for tm in transformation_matrices]
    z_coords = [tm[2, 3] for tm in transformation_matrices]

    # Direction vectors of X, Y, and Z axes
    x_axis_vectors = np.array([[1, 0, 0] @ tm[:3, :3] for tm in transformation_matrices])
    y_axis_vectors = np.array([[0, 1, 0] @ tm[:3, :3] for tm in transformation_matrices])
    z_axis_vectors = np.array([[0, 0, 1] @ tm[:3, :3] for tm in transformation_matrices])

    # Create figure
    fig = go.Figure()

    # Plot links
    fig.add_trace(go.Scatter3d(x=x_coords, y=y_coords, z=z_coords,
                               mode='lines', name='Robot Arm'))
    
    # Plot z-axis directions
    # for z_axis_vector, x, y, z in z_axis_vectors, x_coords, y_coords, z_coords:
    #     fig.add_trace(go.Scatter3d(x=[x, x + z_axis_vector[0]], y=[y, y + z_axis_vector[1]], z=[z, z + z_axis_vector[2]],
    #                                mode='lines', line=dict(color='red'), name='Z-Axis'))

    # Plot joints
    #fig.add_trace(go.Scatter3d(x=x_coords, y=y_coords, z=z_coords, mode='markers', marker=dict(color='red'), name='Joints'))


    # Set layout
    fig.update_layout(title='3D Robot Arm Configuration',
                      scene=dict(aspectmode='data'),
                      showlegend=True)

    # Show plot
    fig.show()

if __name__ == "__main__":
    # Example angles (in degrees)

    THETA1 = 45
    THETA2 = 45
    THETA3 = 45
    THETA4 = 0

    # Plot 3D robot arm
    plot_3d_robot_arm(THETA1, THETA2, THETA3, THETA4)
