#!/usr/bin/env python3
import os

SIMULATION = False

VERTICES_NUM = 4
# new_doublecircle, new_rectangle, test_circle
SAVE_CSV = True
DATA_BASE_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'vertices_data')
#FOLDER = 'prova_ur_prova.bag' #which data you want to unzip from bag


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