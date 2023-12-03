"""This file uses the forward kinematics model to display our virtual robot arm in a static position"""

from forward_kinematics import generate_link_coordinates
from kinematic_constants import *
import plotly.graph_objects as go
import numpy as np
import pandas as pd

def plot_3d_robot_arm(theta1, theta2, theta3, theta4):
    """
    takes in the angles of the robot arm and plots its configuration using plotly
    """

    # Calculate forward kinematics
    link_coordinates = generate_link_coordinates(theta1, theta2, theta3, theta4)


    # Create figure
    fig = go.Figure()

    # Plot links
    fig.add_trace(go.Scatter3d(x=link_coordinates['x'], y=link_coordinates['y'], z=link_coordinates['z'],
                               mode='lines+markers', name='Robot Arm'))

    # Plot joints
    fig.add_trace(go.Scatter3d(x=link_coordinates['x'], y=link_coordinates['y'], z=link_coordinates['z'],
                               mode='markers', marker=dict(color='red'), name='Joints'))

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
