import xml.etree.ElementTree as ET
import os
import re
import numpy as np
from SVG_to_coords import *
import plotly.express as px

def coords_to_toolpath(coords):
    total_toolpath = []
    for i in range(0, len(coords)):
        if coords[i][0] == 'M' or coords[i][0] == 'm':
            toolpath = []
            xy_coords = []
            xy_coords.append(coords[i][1])
            xy_coords.append(coords[i][2])
            toolpath.append(xy_coords)
        else:
            xy_coords = []
            xy_coords.append(coords[i][1])
            xy_coords.append(coords[i][2])
            toolpath.append(xy_coords)

        if coords[i][0] == 'M' or coords[i][0] == 'm':
            total_toolpath.append(toolpath)
    return total_toolpath

if __name__ == "__main__":
    current_working_directory = os.getcwd()
    #file_name = input("What file do you want to use?\n")
    file_name = "spiral_1701380745860"
    file_path = current_working_directory + '\\Code\\Python\\svgs\\' + file_name + '.svg'
    coords = parse_svg(file_path)
    toolpath = coords_to_toolpath(coords)
    #print(toolpath)
    # file_object = open("example.txt", "w")
    # file_object.write(str(toolpath))
    xs = []
    ys = []
    for i in range(0, len(toolpath)):
        for j in range(0, len(toolpath[i])):
            xs.append(float(toolpath[i][j][0]))
            ys.append(float(toolpath[i][j][1]))
    #print(xs)
    #print(ys)
    fig = px.line(x=xs, y=ys)
    fig.update_yaxes(scaleanchor="x",scaleratio=1)
    fig.show()