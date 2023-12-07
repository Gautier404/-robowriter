"""
This file contains a function and tests for processing a scaled 2D drawing into a interpolated
3D cartesian toolpath & angular toolpath for the end deffector of our robot arm.
"""

import numpy as np
from constants import PEN_LIFT_MM, MAX_STEP_MM, HOME_POSITION_CARTESIAN, TABLE_HEIGHT_MM
from inverse_kinematics import generate_link_angles, DRAWING_BOUNDS
from tqdm import tqdm
from fit_path import fit_path


def interpolate_toolpath(toolpath: np.array)-> np.array:
    """
    interpolate between points in a toolpath to create a smoother toolpath that has
    a maximum step distance of MAX_STEP_MM between points. Also clip points that are too close
    together.
    
    parameters:
        toolpath: a 2D numpy array of the x, y, z coordinates of the pen tip in mm 
            in the base coordinate frame. ex: [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]]
    
    returns:
        a 2D numpy array of the smoothed x, y, z coordinates of the pen tip in mm 
        in the base coordinate frame. ex: [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]]
    """
    max_step_squared = MAX_STEP_MM**2
    i = 1
    while i < toolpath.shape[0]:
        # calculate the distance between the current point and the previous point
        distance_squared = np.sum((toolpath[i] - toolpath[i-1])**2)

        # if the distance between the current point and the previous point is less than the max step clip the point
        if distance_squared < max_step_squared:
            toolpath = np.delete(toolpath, i, 0)
            i -= 1

        # if the distance between the current point and the previous point is greater than the max step interpolate between them
        elif distance_squared > max_step_squared:
            num_points = int(np.ceil(np.sqrt(distance_squared) / MAX_STEP_MM))
            interpolated_points = np.linspace(toolpath[i-1], toolpath[i], num_points+1)
            toolpath = np.insert(toolpath, i, interpolated_points, axis=0)
            i += num_points
        
        i += 1
    
    return toolpath



    # # initialize interpolated toolpath to be same size as toolpath
    # interpolated_toolpath_length = 100000
    # interpolated_toolpath = np.zeros((interpolated_toolpath_length,3))
    # interpolated_toolpaths = [interpolated_toolpath] # list of interpolated toolpaths
    # interpolated_toolpath_loc = 0 # index of the next point in the interpolated toolpath
    # last_point = toolpath[0]
    # max_step_squared = MAX_STEP_MM**2

    # for i in tqdm(range(1, toolpath.shape[0])):
    #     # calculate the distance between the current point and the previous point
    #     distance_squared = np.sum((toolpath[i] - last_point)**2)


    #     # if the distance between the current point and the previous point is greater than the max step interpolate between them
    #     if distance_squared > max_step_squared:
    #         # calculate the number of points to interpolate between the current point and the previous point
    #         num_points = int(np.ceil(np.sqrt(distance_squared) / MAX_STEP_MM))

    #         # calculate the step size for each dimension
    #         step_size = (toolpath[i] - last_point) / num_points

    #         # interpolate between the current point and the previous point
    #         for j in tqdm(range(num_points)):
    #             last_point += step_size
    #             # if index is out of bounds add a new interpolated toolpath
    #             if interpolated_toolpath_loc >= interpolated_toolpath_length:
    #                 interpolated_toolpath = np.zeros((interpolated_toolpath_length,3))
    #                 interpolated_toolpaths.append(interpolated_toolpath)
    #                 interpolated_toolpath_loc = 0

    #             interpolated_toolpath[interpolated_toolpath_loc] = last_point
    #             interpolated_toolpath_loc += 1

    #     # if the distance between the current point and the previous point is less than the max step clip the point
    #     elif distance_squared < max_step_squared:
    #         for j in range(i, toolpath.shape[0]):
    #             # calculate the distance between the current point and the previous point
    #             distance_squared = np.sum((toolpath[j] - last_point)**2)
    #             if distance_squared > MAX_STEP_MM:
    #                 i = j
    #                 break
    
    # remove information after the last point (interpolated_toolpath_loc is the index of the last point + 1)
    interpolated_toolpath = interpolated_toolpath[:interpolated_toolpath_loc]

    return np.concatenate(interpolated_toolpaths)


def generate_cartesian_toolpath(toolpath: np.array)-> np.array:
    """generate a 3D cartesian toolpath from a scaled 2D toolpath.
    
    parameters:
        toolpath: a list of 2D arrays of the x, y coordinates of the pen tip in mm 
            in the base coordinate frame. ex: [[[x1, y1], [x2, y2]], [[x3, y3], [x4, y4]]]
    
    returns:
        a 3D numpy array of the x, y, z coordinates of the pen tip in mm 
        in the base coordinate frame. ex: [[x1, y1, z1], [x2, y2, z2]], [x3, y3, z3], [x4, y4, z4]]
    """

    # initialize empty array for 3d toolpath
    toolpath_3d = np.array(HOME_POSITION_CARTESIAN, dtype=float) # start at home position

    # add in z coordinates and the movement between drawing positions
    for path in tqdm(toolpath):
        # add z coordinates to each path
        zeros_column = np.zeros((path.shape[0], 1)) + TABLE_HEIGHT_MM
        path_3d = np.hstack((path, zeros_column))


        # add points before & after the path where the pen is lifted off the paper
        point_before = path_3d[0,:] + [0, 0, PEN_LIFT_MM]
        point_after = path_3d[-1,:] + [0, 0, PEN_LIFT_MM]
        path_3d = np.vstack((point_before, path_3d, point_after))
        
        # append the 3d path to the 3d toolpath
        toolpath_3d = np.vstack((toolpath_3d, path_3d))

    
    # end at home position
    toolpath_3d = np.vstack((toolpath_3d, HOME_POSITION_CARTESIAN))

    # interpolate between points/cut out points that are too close together
    interpolated_toolpath = interpolate_toolpath(toolpath_3d)
    return interpolated_toolpath

def generate_angular_toolpath(cartesian_toolpath: np.array):
    """
    Outputs an array of link angles of the robot given a target pen tip position.
    
    parameters:
        cartesian_toolpath: a 2D array of the x, y, z coordinates of the pen tip in mm 
            in the base coordinate frame.
    """
    angular_toolpath = np.zeros((len(cartesian_toolpath), 4))
    for i, position in enumerate(cartesian_toolpath):
        angular_toolpath[i] = generate_link_angles(position)
    return angular_toolpath


if __name__ == "__main__":
    toolpath = np.array([[[0, 0], [100, 100], [100, 200], [200, 200]]], dtype=float)
    scaled_toolpath = fit_path(toolpath, DRAWING_BOUNDS)
    interpolated_toolpath = generate_cartesian_toolpath(toolpath)

    # plot the original and interploated toolpath in 3D using plotly
    import plotly.graph_objects as go

    fig = go.Figure()
    fig.add_trace(go.Scatter3d(x=interpolated_toolpath[:,0], y=interpolated_toolpath[:,1], z=interpolated_toolpath[:,2], mode='markers', name = 'Interpolated Toolpath', marker=dict(color="blue", size=2)))
    fig.add_trace(go.Scatter3d(x=toolpath[:,0], y=toolpath[:,1], z=np.zeros(toolpath.shape[0]), mode='markers', name = 'Original Toolpath'))
    fig.show()

    # test generate angular toolpath function
    # angular_toolpath = generate_angular_toolpath(interpolated_toolpath)
    # print(angular_toolpath)