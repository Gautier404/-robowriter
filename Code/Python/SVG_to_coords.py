import xml.etree.ElementTree as ET
import os
import re
import numpy as np

def find_next_letter(text, position, cache):
    for i in range(position + 1, len(text)):
        char = text[i]
        if char in cache:
            return i
    return -1  # If no next letter found

def find_next_comma(text, position):
    for i in range(position + 1, len(text)):
        char = text[i]
        if char == ',' or char == ' ':
            return i
    return -1

def read_path(d):
    commands = ['M', 'm', 'L', 'l', 'H', 'h', 'V', 'v', 'C', 'c', 'S', 's', 'Q', 'q', 'T', 't', 'A', 'a', 'Z', 'z']
    command_matrix = []
    current_path_start_coords = [0,0]
    #pylint: disable=C0200
    #print(len(d))
    num_commands = 0
    i = 0
    for i in range(0, len(d)):
        #print(d[i])
        match d[i]:
            case 'M':
                M_array = []
                M_array.append("M")
                comma_index = find_next_comma(d, i)
                x_coord = d[i+1:comma_index]
                M_array.append(x_coord)
                current_path_start_coords[0] = x_coord
                next_letter = find_next_letter(d, comma_index + 1, commands)
                y_coord = d[comma_index+1:next_letter]
                M_array.append(y_coord)
                current_path_start_coords[1] = y_coord
                command_matrix.append(M_array)
                num_commands += 1
            case 'm':
                m_array = []
                m_array.append("m")
                comma_index = find_next_comma(d, i)
                x_coord = d[i+1:comma_index]
                m_array.append(x_coord)
                current_path_start_coords[0] = x_coord
                next_letter = find_next_letter(d, comma_index + 1, commands)
                y_coord = d[comma_index+1:next_letter]
                m_array.append(y_coord)
                current_path_start_coords[1] = y_coord
                command_matrix.append(m_array)
                num_commands += 1
            case 'L':
                L_array = []
                L_array.append("L")
                comma_index = find_next_comma(d, i)
                x_coord = d[i+1:comma_index]
                L_array.append(x_coord)
                next_letter = find_next_letter(d, comma_index + 1, commands)
                y_coord = d[comma_index+1:next_letter]
                L_array.append(y_coord)
                command_matrix.append(L_array)
                num_commands += 1
            case 'l':
                l_array = []
                l_array.append("l")
                comma_index = find_next_comma(d, i)
                x_coord = d[i+1:comma_index]
                l_array.append(x_coord)
                next_letter = find_next_letter(d, comma_index + 1, commands)
                y_coord = d[comma_index+1:next_letter]
                l_array.append(y_coord)
                command_matrix.append(l_array)
                num_commands += 1
            case 'H':
                H_array = []
                H_array.append("H")
                next_letter = find_next_letter(d, i, commands)
                x_coord = d[i+1:next_letter]
                H_array.append(x_coord)
                y_coord = command_matrix[num_commands - 1][2]
                H_array.append(y_coord)
                command_matrix.append(H_array)
                num_commands += 1
            case 'h':
                h_array = []
                h_array.append("h")
                next_letter = find_next_letter(d, i, commands)
                x_coord = d[i+1:next_letter]
                h_array.append(x_coord)
                y_coord = command_matrix[num_commands - 1][2]
                h_array.append(y_coord)
                command_matrix.append(h_array)
                num_commands += 1
            case 'V':
                V_array = []
                V_array.append("V")
                x_coord = command_matrix[num_commands - 1][1]
                V_array.append(x_coord)
                next_letter = find_next_letter(d, i, commands)
                y_coord = d[i+1:next_letter]
                V_array.append(y_coord)
                command_matrix.append(V_array)
                num_commands += 1
            case 'v':
                v_array = []
                v_array.append("v")
                x_coord = command_matrix[num_commands - 1][1]
                v_array.append(x_coord)
                next_letter = find_next_letter(d, i, commands)
                y_coord = d[i+1:next_letter]
                v_array.append(y_coord)
                command_matrix.append(v_array)
                num_commands += 1
            case 'C':
                C_array = []
                C_array.append("C")
                comma_1 = find_next_comma(d, i)
                comma_2 = find_next_comma(d, comma_1)
                comma_3 = find_next_comma(d, comma_2)
                comma_4 = find_next_comma(d, comma_3)
                comma_5 = find_next_comma(d, comma_4)
                x_coord = d[comma_4 + 1:comma_5]
                C_array.append(x_coord)
                next_letter = find_next_letter(d, comma_5 + 1, commands)
                y_coord = d[comma_5 + 1: next_letter]
                C_array.append(y_coord)
                command_matrix.append(C_array)
                num_commands += 1
            case 'c':
                c_array = []
                c_array.append("c")
                comma_1 = find_next_comma(d, i)
                comma_2 = find_next_comma(d, comma_1)
                comma_3 = find_next_comma(d, comma_2)
                comma_4 = find_next_comma(d, comma_3)
                comma_5 = find_next_comma(d, comma_4)
                x_coord = d[comma_4 + 1:comma_5]
                c_array.append(x_coord)
                next_letter = find_next_letter(d, comma_5 + 1, commands)
                y_coord = d[comma_5 + 1: next_letter]
                c_array.append(y_coord)
                command_matrix.append(c_array)
                num_commands += 1
            case 'S':
                S_array = []
                S_array.append("S")
                comma_1 = find_next_comma(d, i)
                comma_2 = find_next_comma(d, comma_1)
                comma_3 = find_next_comma(d, comma_2)
                x_coord = d[comma_2 + 1:comma_3]
                S_array.append(x_coord)
                next_letter = find_next_letter(d, comma_3 + 1, commands)
                y_coord = d[comma_3 + 1: next_letter]
                S_array.append(y_coord)
                command_matrix.append(S_array)
                num_commands += 1
            case 's':
                s_array = []
                s_array.append("s")
                comma_1 = find_next_comma(d, i)
                comma_2 = find_next_comma(d, comma_1)
                comma_3 = find_next_comma(d, comma_2)
                x_coord = d[comma_2 + 1:comma_3]
                s_array.append(x_coord)
                next_letter = find_next_letter(d, comma_3 + 1, commands)
                y_coord = d[comma_3 + 1: next_letter]
                s_array.append(y_coord)
                command_matrix.append(s_array)
                num_commands += 1
            case 'Q':
                Q_array = []
                Q_array.append("S")
                comma_1 = find_next_comma(d, i)
                comma_2 = find_next_comma(d, comma_1)
                comma_3 = find_next_comma(d, comma_2)
                x_coord = d[comma_2 + 1:comma_3]
                Q_array.append(x_coord)
                next_letter = find_next_letter(d, comma_3 + 1, commands)
                y_coord = d[comma_3 + 1: next_letter]
                Q_array.append(y_coord)
                command_matrix.append(Q_array)
                num_commands += 1
            case 'q':
                q_array = []
                q_array.append("q")
                comma_1 = find_next_comma(d, i)
                comma_2 = find_next_comma(d, comma_1)
                comma_3 = find_next_comma(d, comma_2)
                x_coord = d[comma_2 + 1:comma_3]
                q_array.append(x_coord)
                next_letter = find_next_letter(d, comma_3 + 1, commands)
                y_coord = d[comma_3 + 1: next_letter]
                q_array.append(y_coord)
                command_matrix.append(q_array)
                num_commands += 1
            case 'T':
                T_array = []
                T_array.append("T")
                comma_index = find_next_comma(d, i)
                x_coord = d[i+1:comma_index]
                T_array.append(x_coord)
                next_letter = find_next_letter(d, comma_index + 1, commands)
                y_coord = d[comma_index+1:next_letter]
                T_array.append(y_coord)
                command_matrix.append(T_array)
                num_commands += 1
            case 't':
                t_array = []
                t_array.append("t")
                comma_index = find_next_comma(d, i)
                x_coord = d[i+1:comma_index]
                t_array.append(x_coord)
                next_letter = find_next_letter(d, comma_index + 1, commands)
                y_coord = d[comma_index+1:next_letter]
                t_array.append(y_coord)
                command_matrix.append(t_array)
                num_commands += 1
            case 'A':
                A_array = []
                A_array.append("A")
                comma_1 = find_next_comma(d, i)
                comma_2 = find_next_comma(d, comma_1)
                comma_3 = find_next_comma(d, comma_2)
                comma_4 = find_next_comma(d, comma_3)
                comma_5 = find_next_comma(d, comma_4)
                comma_6 = find_next_comma(d, comma_5)
                x_coord = d[comma_5 + 1:comma_6]
                A_array.append(x_coord)
                next_letter = find_next_letter(d, comma_6 + 1, commands)
                y_coord = d[comma_6 + 1: next_letter]
                A_array.append(y_coord)
                command_matrix.append(A_array)
                num_commands += 1
            case 'a':
                a_array = []
                a_array.append("a")
                comma_1 = find_next_comma(d, i)
                comma_2 = find_next_comma(d, comma_1)
                comma_3 = find_next_comma(d, comma_2)
                comma_4 = find_next_comma(d, comma_3)
                comma_5 = find_next_comma(d, comma_4)
                comma_6 = find_next_comma(d, comma_5)
                x_coord = d[comma_5 + 1:comma_6]
                a_array.append(x_coord)
                next_letter = find_next_letter(d, comma_6 + 1, commands)
                y_coord = d[comma_6 + 1: next_letter]
                a_array.append(y_coord)
                command_matrix.append(a_array)
                num_commands += 1
            case 'Z':
                Z_array = []
                Z_array.append("Z")
                Z_array.append(current_path_start_coords[0])
                Z_array.append(current_path_start_coords[1])
                command_matrix.append(Z_array)
                num_commands += 1
            case 'z':
                z_array = []
                z_array.append("z")
                z_array.append(current_path_start_coords[0])
                z_array.append(current_path_start_coords[1])
                command_matrix.append(z_array)
                num_commands += 1
            case _:
                continue
    print(command_matrix)
    return command_matrix

def parse_svg(svg_file):
    tree = ET.parse(svg_file)
    #print(tree)
    root = tree.getroot()
    #print(root)

    # Define SVG namespace
    svg_namespace = {'svg': 'http://www.w3.org/2000/svg'}

    # Find all path elements
    paths = root.findall('.//svg:path', namespaces=svg_namespace)

    for path in paths:
        d_attrib = path.attrib.get('d', '')
        command_matrix = read_path(d_attrib)
    return command_matrix
    


if __name__ == "__main__":
    current_working_directory = os.getcwd()
    #file_name = input("What file do you want to use?\n")
    file_name = "rectangle"
    file_path = current_working_directory + '\\robowriter\\Code\\Python\\svgs\\' + file_name + '.svg'
    print(parse_svg(file_path))
    #convert_to_toolpath(paths)