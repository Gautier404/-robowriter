"""This file contains code for animating the robot arm as it moves through a series of angles."""
import numpy as np
import plotly.graph_objects as go
from forward_kinematics import generate_link_coordinates

# Time steps
time_steps = np.linspace(0, 180, 20)  # Adjust the range and step size as needed


def animate_arm(angular_toolpath: np.array, cartesian_toolpath: np.array = None) -> None:
    """
    Generates an animation of the robot arm moving through a series of angles.

    parameters:
        angular_toolpath: a two dimensional array of angles that represent the position 
            of each servo for a given timestep.
    """
    # If no cartesian toolpath is specified , create an single point at 0, 0, 0.
    if cartesian_toolpath is None:
        cartesian_toolpath = np.array([[0],[0],[0]])

    # Initialize empty arrays for x, y, and z coordinates
    x = np.zeros((len(angular_toolpath), 7), dtype=float)
    y = np.zeros((len(angular_toolpath), 7), dtype=float)
    z = np.zeros((len(angular_toolpath), 7), dtype=float)

    # Generate link coordinates for each time step.
    for i, step in enumerate(angular_toolpath):
        link_coordinates = generate_link_coordinates(step[0], step[1], step[2], step[3])
        x[i] = link_coordinates["x"]
        y[i] = link_coordinates["y"]
        z[i] = link_coordinates["z"]

    # find maximum & minimum x, y & z values of all coordinates for axis ranges.
    padding = 50
    max_x = max([max(i) for i in x]) + padding
    min_x = min([min(i) for i in x]) - padding
    max_y = max([max(i) for i in y]) + padding
    min_y = min([min(i) for i in y]) - padding
    max_z = max([max(i) for i in z]) + padding
    min_z = min([min(i) for i in z])

    # define aspect ratios such that the plot's x, y, & z axis have equal units.
    range_x = max_x - min_x
    range_y = max_y - min_y
    range_z = max_z - min_z
    max_range = max(range_x, range_y, range_z)
    x_aspect = range_x / max_range
    y_aspect = range_y / max_range
    z_aspect = range_z / max_range

    # Create layout
    animation_layout = go.Layout(
                scene=dict(
                    xaxis=dict(range=[min_x, max_x], autorange=False),
                    yaxis=dict(range=[min_y, max_y], autorange=False),
                    zaxis=dict(range=[min_z, max_z], autorange=False),
                    aspectmode="manual",
                    aspectratio=dict(x=x_aspect, y=y_aspect, z=z_aspect),
                )
    )

    # Create coordinate list for the actual toolpath (last joint of the robot)
    toolpath_x = x[:,6].T
    toolpath_y = y[:,6].T
    toolpath_z = z[:,6].T

    # Create figure
    fig = go.Figure(
        data=[
            go.Scatter3d(
                x=x[0],
                y=y[0],
                z=z[0],
                mode="markers",
                marker=dict(color="red", size=5),
                name="joints",
            ),
            go.Scatter3d(
                x=x[0],
                y=y[0],
                z=z[0],
                mode="lines",
                line=dict(color="blue", width=5),
                name="links",
            ),
            go.Scatter3d(
                x = cartesian_toolpath[0],
                y = cartesian_toolpath[1],
                z = cartesian_toolpath[2],
                name = "desired_toolpath",
                mode="markers",
                marker=dict(color="green", size=2),
            ),
            go.Scatter3d(
                x = toolpath_x[0:0],
                y = toolpath_y[0:0],
                z = toolpath_z[0:0],
                name = "actual_toolpath",
                mode="markers",
                marker=dict(color="black", size=2),
            )
        ],
        layout= animation_layout
    )

    # Frames
    frames = [
        go.Frame(
            data=[
                go.Scatter3d(
                    x=x[k],
                    y=y[k],
                    z=z[k],
                    mode="markers",
                    marker=dict(color="red", size=10),
                    name="joints",
                ),
                go.Scatter3d(
                    x=x[k],
                    y=y[k],
                    z=z[k],
                    mode="lines",
                    line=dict(color="blue", width=5),
                    name="links",
                ),
                go.Scatter3d(
                    x = cartesian_toolpath[0],
                    y = cartesian_toolpath[1],
                    z = cartesian_toolpath[2],
                    name = "desired_toolpath",
                    mode="markers",
                    marker=dict(color="green", size=2),
                ),
                go.Scatter3d(
                    x = toolpath_x[:k],
                    y = toolpath_y[:k],
                    z = toolpath_z[:k],
                    name = "actual_toolpath",
                    mode="markers",
                    marker=dict(color="black", size=2),
                )
            ],
            # traces= [0, 1], # corresponds to which traces should be displayed...
            name=f"frame{k}",
            layout=animation_layout,
        )
        for k in range(len(x))
    ]

    fig.update(frames=frames)

    def frame_args(duration):
        return {
            "frame": {"duration": duration},
            "mode": "immediate",
            "fromcurrent": True,
            "transition": {"duration": duration, "easing": "linear"},
        }

    sliders = [
        {
            "pad": {"b": 10, "t": 60},
            "len": 0.9,
            "x": 0.1,
            "y": 0,
            "steps": [
                {
                    "args": [[f.name], frame_args(0)],
                    "label": str(k),
                    "method": "animate",
                }
                for k, f in enumerate(fig.frames)
            ],
        }
    ]

    fig.update_layout(
        updatemenus=[
            {
                "buttons": [
                    {
                        "args": [None, frame_args(50)],
                        "label": "Play",
                        "method": "animate",
                    },
                    {
                        "args": [[None], frame_args(0)],
                        "label": "Pause",
                        "method": "animate",
                    },
                ],
                "direction": "left",
                "pad": {"r": 10, "t": 70},
                "type": "buttons",
                "x": 0.1,
                "y": 0,
            }
        ],
        sliders=sliders,
    )

    fig.update_layout(sliders=sliders)
    fig.show()


if __name__ == "__main__":
    anglular_toolpath = np.array([
        np.linspace(0, 0, 40),
        np.linspace(45, 45, 40),
        np.linspace(45, 45, 40),
        np.linspace(0, 45, 40)
    ])
    desired_toolpath = np.array([
        np.linspace(0, 100, 40),
        np.linspace(0, 0, 40),
        np.linspace(0, 0, 40)
    ])


    anglular_toolpath = anglular_toolpath.T

    animate_arm(anglular_toolpath, desired_toolpath)
