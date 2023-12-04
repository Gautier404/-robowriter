"""This page contains code to simulate the toolpath of the robot arm as it moves through a series of angles."""
import numpy as np
from generate_toolpath import generate_cartesian_toolpath, generate_angular_toolpath
from animate_arm import animate_arm

input_toolpath = np.array([[[0, 200], [100, 200]], [[200, 100], [200, 200]]], dtype=float)
consolidated_input_toolpath = np.concatenate(input_toolpath)
consolidated_input_toolpath_3d = np.hstack((consolidated_input_toolpath, np.zeros((consolidated_input_toolpath.shape[0], 1))))
print(consolidated_input_toolpath_3d)
interpolated_toolpath = generate_cartesian_toolpath(input_toolpath)
angular_toolpath = generate_angular_toolpath(interpolated_toolpath)

#animate_arm(angular_toolpath, consolidated_input_toolpath_3d)