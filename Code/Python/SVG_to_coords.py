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
    for i in range(position + 2, len(text)):
        char = text[i]
        if char == '-':
            return i-1
        if char == ',' or char == ' ':
            return i
    return -1

def read_path2(path: str, decimal_places=3):
    command_matrix = []
    current_line = []
    current_num = ''
    is_decimal = False
    numbers_after_decimal = 0

    for char in path:
        match char:
            case 'M'| 'm' | 'L' | 'l' | 'H' | 'h' | 'V' | 'v' | 'C' | 'c' | 'S' | 's' | 'Q' | 'q' | 'T' | 't' | 'A' | 'a' | 'Z' | 'z':
                if current_num:
                    current_line.append(current_num)
                    is_decimal = False
                    current_num = ''

                if current_line:
                    command_matrix.append(current_line)
                current_line = [char]
            case ' ' | ',':
                if current_num:
                    current_line.append(current_num)
                    is_decimal = False
                    current_num = ''
            case '-':
                if current_num:
                    current_line.append(current_num)
                is_decimal = False
                current_num = '-'
            case '.':
                if current_num and is_decimal:
                    current_line.append(current_num)
                    current_num = '.'
                else:
                    current_num += '.'
                numbers_after_decimal = 0
                is_decimal = True
            case _:
                current_num += char
                numbers_after_decimal += 1
                if is_decimal and (numbers_after_decimal >= decimal_places):
                    current_line.append(current_num)
                    is_decimal = False
                    current_num = ''

    if current_num:
        current_line.append(current_num)

    if current_line:
        command_matrix.append(current_line)

    return command_matrix

def parse_svg(svg_file, decimal_places=3):
    tree = ET.parse(svg_file)
    #print(tree)
    root = tree.getroot()
    #print(root)

    # Define SVG namespace
    svg_namespace = {'svg': 'http://www.w3.org/2000/svg'}

    # Find all path elements
    paths = root.findall('.//svg:path', namespaces=svg_namespace)
    command_matrix = []
    for path in paths:
        d_attrib = path.attrib.get('d', '')
        command_matrix.append(read_path2(d_attrib, decimal_places))
    return command_matrix

if __name__ == "__main__":
    current_working_directory = os.getcwd()
    #file_name = input("What file do you want to use?\n")
    file_name = "hello_world"
    file_path = current_working_directory + '\\Code\\Python\\svgs\\' + file_name + '.svg'
    print(parse_svg(file_path, 2)[0])
    #convert_to_toolpath(paths)

    #test_string = "M501.333,96H10.667C4.779,96,0,100.779,0,106.667v298.667C0,411.221,4.779,416,10.667,416h490.667c5.888,0,10.667,4.779,-10.667,-10.667V106.667C512,100.779,507.221,96,501.333,96z M490.667,394.667H21.333V117.333h469.333V394.667z"
    #print(read_path2(test_string))