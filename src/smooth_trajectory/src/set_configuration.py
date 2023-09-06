#!/usr/bin/env python3

import os

SETTING_GUI = False

if SETTING_GUI:
    dirname = os.path.dirname(__file__)
    file_path = os.path.join(dirname, 'test.txt')
    file_path = os.path.join(os.path.dirname(os.path.dirname(dirname)), 'rqt_mypkg/src/rqt_mypkg/test.txt')

    file = open(file_path, "r")
        
    lines = []
    myline = file.readline().strip()
    lines.append(myline)
    while myline:
        myline = file.readline().strip()
        lines.append(myline)
    file.close() 

    SIMULATION = lines[4]
    CAD_OBJECT_SIZE = lines[0]# rectangle | rectangle_doublecircle | rectangle_circle | circle | B3 | deep_rectangle_doublecircle
    CAD_OBJECT_START = lines[1]
    CAD_ORIENTATION =  lines[2]# left or right side
    ICP_THRESHOLD = float(lines[3])
    
else:
    SIMULATION = False

    CAD_OBJECT_SIZE = 'rectangle' # rectangle | rectangle_doublecircle | rectangle_circle | circle | B3 | deep_rectangle_doublecircle
    CAD_OBJECT_START = 'z+'
    CAD_ORIENTATION = 'right' # left or right side
    ICP_THRESHOLD = 100
    # 50 for double circle from 3 to 6
TRAJECTORY = 'new_rectangle' # which trajectory you want to work with
# new_doublecircle, new_rectangle, test_circle
SAVE_CSV = False
SAVE_PLOT = False
plot_timing_law = False

DATA_BASE_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data')
FOLDER = 'demo_doub.bag' #which data you want to unzip from bag


#####               RECTANGLE CONFIGURATION                #####
#   z+ right ---> starting from low left side to low right side
#   z+ left ---> starting from up left side to up right side
#   z- right ---> starting from low right side to low left side
#   z- left ---> starting from up right side to up left side

#   x+ right ---> starting from low right side to up right side
#   x+ left ---> starting from low left side to up left side
#   x- right ---> starting from up left side to low left side
#   x- left ---> starting from up right side to low right side
#