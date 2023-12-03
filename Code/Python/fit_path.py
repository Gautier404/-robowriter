"""
This file contains a function and tests for processing a simple 2D drawing into a interpolated and 
scaled cartesian toolpath for the end deffector of our robot arm
"""
import numpy as np

def fit_path(flat_toolpath: np.array, bounds: np.array) -> np.array:
    """fit the 2d toolpath to the bounds of the drawing area using simple bounds.
    
    parameters:
        flat_toolpath_in: a list of 2D arrays of the x, y coordinates of the pen tip in mm 
            in the base coordinate frame. ex: [[[x1, y1], [x2, y2]], [[x3, y3], [x4, y4]]]
        bounds: a 2D array of two x, y coordinates of the corners of a rectangle that represent 
            the bounds of the drawing area. ex: [[x1, y1], [x2, y2]]
    
    returns:
        a numpy array of 2D arrays of the x, y coordinates of the pen tip in mm 
        in the base coordinate frame. ex: [[[x1, y1], [x2, y2]], [[x3, y3], [x4, y4]]]
        Each 2D array represents a path that is drawn without taking the pen off the paper.
    """

    # find the min and max x and y values of the toolpath
    consolidated_toolpath = np.concatenate(flat_toolpath) 
    x_min_tp = min(consolidated_toolpath[:,0])
    x_max_tp = max(consolidated_toolpath[:,0])
    y_min_tp = min(consolidated_toolpath[:,1])
    y_max_tp = max(consolidated_toolpath[:,1])

    # find the min and max x and y values of the drawing area
    x_min = min(bounds[:,0])
    x_max = max(bounds[:,0])
    y_min = min(bounds[:,1])
    y_max = max(bounds[:,1])

    # calculate the scale factor for the toolpath
    x_scale = (x_max - x_min) / (x_max_tp - x_min_tp)
    y_scale = (y_max - y_min) / (y_max_tp - y_min_tp)

    # use minimum scalar to keep the aspect ratio of the toolpath the same
    scale = min(x_scale, y_scale)


    # find center of toolpath and drawing area
    x_center_tp = (x_max_tp + x_min_tp) / 2
    y_center_tp = (y_max_tp + y_min_tp) / 2
    center_tp = np.array([x_center_tp, y_center_tp])
    x_center = (x_max + x_min) / 2
    y_center = (y_max + y_min) / 2
    center = np.array([x_center, y_center])

    for path in flat_toolpath:
        path -= center_tp           # center the toolpath on the origin
        path *= [scale, scale]  # scale the toolpath
        path += center              # center the toolpath in the drawing area

    return flat_toolpath


if __name__ == "__main__":
    bounds = np.array([[0, 0], [10, 10]], dtype=float)
    flat_toolpath_in = np.array([[[0, 0], [0, 10]], [[0, -20], [-20, -20]], [[20, 20], [20, 0]]], dtype=float)
    consolidated_toolpath = np.concatenate(flat_toolpath_in) 
    new_path = fit_path(flat_toolpath_in, bounds)
    print(new_path)

    # using plotly show the original toolpath, the bounds, and the scaled toolpath
    import plotly.graph_objects as go

    # Create figure
    fig = go.Figure()

    # Plot original toolpath
    fig.add_trace(go.Scatter(x=consolidated_toolpath[:,0], y=consolidated_toolpath[:,1], mode='markers', name='Original Toolpath'))

    # Plot bounds
    fig.add_trace(go.Scatter(x=bounds[:,0], y=bounds[:,1], mode='markers', name='Bounds'))

    # Plot scaled toolpath
    consolidated_scaled_toolpath = np.concatenate(new_path)
    fig.add_trace(go.Scatter(x=consolidated_scaled_toolpath[:,0], y=consolidated_scaled_toolpath[:,1], mode='markers', name='Scaled Toolpath'))

    # Set layout
    fig.update_layout(title='Toolpath Scaling',
                      showlegend=True)
    
    # Show plot
    fig.show()
