import numpy as np
import plotly.graph_objects as go
from forward_kinematics import forward_kinematics

def plot_3d_robot_arm_animation():
    # pylint: disable=E1136  # pylint/issues/3139
    
    # Example angles (in degrees)
    theta1 = 0
    theta2 = 0
    theta3 = 0
    theta4 = 0

    # Create figure
    fig = go.Figure()

    # Set layout
    fig.update_layout(title='3D Robot Arm Configuration',
                      scene=dict(aspectmode='data'),
                      showlegend=True)

    frames = 36  # Number of frames for animation

    # Add frames to the figure
    for frame in range(frames):
        theta1 += 10  # Increment angle for each frame
        transformation_matrices = forward_kinematics(theta1, theta2, theta3, theta4)

        # Joint positions
        x_coords = [tm[0, 3] for tm in transformation_matrices]
        y_coords = [tm[1, 3] for tm in transformation_matrices]
        z_coords = [tm[2, 3] for tm in transformation_matrices]

        # Add scatter trace for links
        fig.add_trace(go.Scatter3d(x=x_coords, y=y_coords, z=z_coords,
                                   mode='lines+markers',
                                   name=f'Frame {frame}'))

    # Set up animation settings
    animation_settings = dict(frame=dict(duration=100, redraw=True), fromcurrent=True)

    # Update layout with animation settings
    fig.update_layout(updatemenus=[dict(type='buttons', showactive=False, buttons=[dict(label='Play',
                                            method='animate', args=[None, animation_settings])])])

    # Show plot
    fig.show()

if __name__ == "__main__":
    plot_3d_robot_arm_animation()