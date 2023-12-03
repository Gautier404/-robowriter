"""This file contains code for animating the robot arm as it moves through a series of angles."""
import numpy as np
import plotly.graph_objects as go
from forward_kinematics import generate_link_coordinates

# Time steps
time_steps = np.linspace(0, 180, 20)  # Adjust the range and step size as needed


def animate_arm(angular_toolpath: np.array) -> None:
    """
    Generates an animation of the robot arm moving through a series of angles.

    parameters:
        angular_toolpath: a two dimensional array of angles that represent the position 
            of each servo for a given timestep.
    """
    # Initialize empty lists for x, y, and z coordinates
    x = []
    y = []
    z = []

    # Generate link coordinates for each time step.
    for step in angular_toolpath:
        link_coordinates = generate_link_coordinates(
            step[0],
            step[1],
            step[2],
            step[3],
        )
        x.append(link_coordinates["x"])
        y.append(link_coordinates["y"])
        z.append(link_coordinates["z"])

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

    # Create figure
    fig = go.Figure(
        data=[
            go.Scatter3d(
                x=x[0],
                y=y[0],
                z=z[0],
                mode="markers",
                marker=dict(color="red", size=10),
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
    toolpath = np.array([
        np.linspace(0, 0, 40),
        np.linspace(45, 45, 40),
        np.linspace(45, 45, 40),
        np.linspace(0, 45, 40)
    ])
    toolpath = toolpath.T

    animate_arm(toolpath)