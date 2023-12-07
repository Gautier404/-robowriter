import xml.etree.ElementTree as ET
import os
import re
import numpy as np
from SVG_to_coords import *
import plotly.express as px
import bezier
from constants import *

def get_last_x(arr):
    if arr.ndim == 1:
        # 1D array
        return arr[-2]
    elif arr.ndim == 2:
        # 2D array
        return arr[-1, -2]
    else:
        # Handle arrays with more than 2 dimensions as needed
        raise ValueError("Arrays with more than 2 dimensions are not supported.")
    
def get_last_y(arr):
    if arr.ndim == 1:
        # 1D array
        return arr[-1]
    elif arr.ndim == 2:
        # 2D array
        return arr[-1, -1]
    else:
        # Handle arrays with more than 2 dimensions as needed
        raise ValueError("Arrays with more than 2 dimensions are not supported.")

def read_path(command_matrix):
    toolpath_set = False
    total_toolpath = []
    for command_path in command_matrix:
        for command_line in command_path:
            match command_line[0]:
                case 'M':
                    if toolpath_set:
                        total_toolpath.append(toolpath)
                    toolpath = np.array([float(command_line[1]), float(command_line[2])], dtype=float)
                    toolpath_set = True
                case 'm':
                    if toolpath_set:
                        total_toolpath.append(toolpath)
                        toolpath = np.array([float(command_line[1]) + get_last_x(total_toolpath[-1]), float(command_line[2]) + get_last_y(total_toolpath[-1])], dtype=float)
                    else:
                        toolpath = np.array([float(command_line[1]), float(command_line[2])], dtype=float)    
                    toolpath_set = True
                case 'L':
                    for i in range(1, len(command_line)-1, 2):
                        toolpath = np.vstack((toolpath, np.array([float(command_line[i]), float(command_line[i+1])])))
                case 'l':
                    for i in range(1, len(command_line)-1, 2):
                        toolpath = np.vstack((toolpath, np.array([float(command_line[i]) + get_last_x(toolpath), float(command_line[i+1]) + get_last_y(toolpath)])))
                case 'H':
                    for i in range(1, len(command_line)):
                        toolpath = np.vstack((toolpath, np.array([float(command_line[i]), get_last_y(toolpath)])))
                case 'h':
                    for i in range(1, len(command_line)):
                        toolpath = np.vstack((toolpath, np.array([float(command_line[i]) + get_last_x(toolpath), get_last_y(toolpath)])))
                case 'V':
                    for i in range(1, len(command_line)):
                        toolpath = np.vstack((toolpath, np.array([get_last_x(toolpath), float(command_line[i])])))
                case 'v':
                    for i in range(1, len(command_line)):
                        toolpath = np.vstack((toolpath, np.array([get_last_x(toolpath), float(command_line[i]) + get_last_y(toolpath)])))
                case 'C':
                    number_of_curves = int((len(command_line) - 1)/6)
                    for i in range(0, number_of_curves):
                        nodes = np.asfortranarray([
                            [get_last_x(toolpath), float(command_line[1+6*i]), float(command_line[3+6*i]), float(command_line[5+6*i])],
                            [get_last_y(toolpath), float(command_line[2+6*i]), float(command_line[4+6*i]), float(command_line[6+6*i])],
                        ])
                        curve = bezier.Curve(nodes, degree=3)
                        for i in range(1, POINTS_IN_CURVE+1):
                            percentage = i/POINTS_IN_CURVE
                            toolpath = np.vstack((toolpath, curve.evaluate(percentage).T))
                case 'c':
                    number_of_curves = int((len(command_line) - 1)/6)
                    for i in range(0, number_of_curves):
                        nodes = np.asfortranarray([
                            [get_last_x(toolpath), float(command_line[1+6*i])+ get_last_x(toolpath), float(command_line[3+6*i])+ get_last_x(toolpath), float(command_line[5+6*i])+ get_last_x(toolpath)],
                            [get_last_y(toolpath), float(command_line[2+6*i])+ get_last_y(toolpath), float(command_line[4+6*i])+ get_last_y(toolpath), float(command_line[6+6*i])+ get_last_y(toolpath)],
                        ])
                        curve = bezier.Curve(nodes, degree=3)
                        for i in range(1, POINTS_IN_CURVE+1):
                            percentage = i/POINTS_IN_CURVE
                            toolpath = np.vstack((toolpath, curve.evaluate(percentage).T))  
                case 'S':
                    toolpath = np.vstack((toolpath, np.array([float(command_line[3]), float(command_line[4])])))
                case 's':
                    toolpath = np.vstack((toolpath, np.array([float(command_line[3]) + get_last_x(toolpath), float(command_line[4]) + get_last_y(toolpath)])))
                case 'Q':
                    toolpath = np.vstack((toolpath, np.array([float(command_line[3]), float(command_line[4])])))
                case 'q':
                    toolpath = np.vstack((toolpath, np.array([float(command_line[3]) + get_last_x(toolpath), float(command_line[4]) + get_last_y(toolpath)])))
                case 'T':
                    toolpath = np.vstack((toolpath, np.array([float(command_line[1]), float(command_line[2])])))
                case 't':
                    toolpath = np.vstack((toolpath, np.array([float(command_line[1]) + get_last_x(toolpath), float(command_line[2]) + get_last_y(toolpath)])))
                case 'A':
                    toolpath = np.vstack((toolpath, np.array([float(command_line[6]), float(command_line[7])])))
                case 'a':
                    toolpath = np.vstack((toolpath, np.array([float(command_line[6]) + get_last_x(toolpath), float(command_line[7]) + get_last_y(toolpath)])))
                case 'Z' | 'z':
                    toolpath = np.vstack((toolpath, toolpath[0]))
                case _:
                    continue
        if toolpath_set:
            total_toolpath.append(toolpath)
            toolpath_set = False

    # flip y axis
    for i, toolpath in enumerate(total_toolpath):
        total_toolpath[i] = toolpath*np.array([1, -1])
    return total_toolpath

# def coords_to_toolpath(coords):
#     total_toolpath = []
#     tp = np.array([])
#     toolpath_set = False
#     for i, command in enumerate(coords):
#         if coords[i][0].lower() == 'm':
#             if toolpath_set:
#                 total_toolpath.append(tp)
#             tp = np.array([float(command[1]), float(command[2])], dtype=float)
#             toolpath_set = True
#         else:
#             tp = np.vstack((tp, np.array([float(command[1]), float(command[2])])))
#     total_toolpath.append(tp)
#     return total_toolpath

if __name__ == "__main__":
    current_working_directory = os.getcwd()
    #file_name = input("What file do you want to use?\n")
    file_name = "hello_world"
    file_path = current_working_directory + '\\Code\\Python\\svgs\\' + file_name + '.svg'
    coords = parse_svg(file_path, 2)
    print(coords)
    toolpath = read_path(coords)
    print(toolpath)
    consolidated_toolpath = np.concatenate(toolpath)
    fig = px.line(x=consolidated_toolpath[:,0], y=consolidated_toolpath[:,1])
    fig.update_yaxes(scaleanchor="x",scaleratio=1)
    fig.show()