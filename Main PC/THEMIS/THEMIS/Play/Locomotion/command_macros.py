#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "August 25, 2025"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Script that holds useful demo macros
'''

from Play.config import *
from Setting.Macros.model_macros import *
from Setting.Macros.locomotion_macros import *


# ==================================================
# ==================================================
# TOP-LEVEL
# ==================================================
# ==================================================

COM_POSITION_X     = 0
COM_POSITION_Y     = 1
COM_POSITION_Z     = 2

BASE_ORIENTATION_X = 3
BASE_ORIENTATION_Y = 4
BASE_ORIENTATION_Z = 5

COM_VELOCITY_X     = 6
COM_VELOCITY_Y     = 7
BASE_YAW_RATE      = 8

FOOT_YAW_RIGHT     = 9
FOOT_YAW_LEFT      = 10

FOOT_CLEARANCE_X   = 11
FOOT_CLEARANCE_Y   = 12

PARAMETER_ID_LIST      = range(13)
PARAMETER_INCREMENT    = [ 0.01,  0.01,  0.01,       1,     1,     1,    0.01,  0.01,     5,       1,     1,    0.01,  0.01]
PARAMETER_DEFAULT      = [ 0.00,  0.00,  0.00,       0,     0,     0,    0.00,  0.00,     0,       0,     0,    0.00,  0.00]
PARAMETER_MAX          = [ 0.02,  0.03,  0.05,      10,    10,    20,    0.20,  0.20,    15,      10,    20,    0.10,  0.10]
PARAMETER_MIN          = [-0.02, -0.03, -0.20,     -10,   -10,   -20,   -0.20, -0.20,   -15,     -20,   -10,   -0.10, -0.10]
PARAMETER_BUTTON_PLUS  = [  'g',   'j',   'l',     'y',   'i',   'p',     'w',   'a',   'q',     'x',   'v',     'm',   '.']
PARAMETER_BUTTON_MINUS = [  'f',   'h',   'k',     't',   'u',   'o',     's',   'd',   'e',     'z',   'c',     'n',   ',']
PARAMETER_TYPE         = ['len', 'len', 'len',   'ang', 'ang', 'ang',   'len', 'len', 'ang',   'ang', 'ang',   'len', 'len']
PARAMETER_RECOVER      = [  'y',   'y',   'y',     'y',   'y',   'y',     'y',   'y',   'y',     'y',   'y',     'y',   'y']

PARAMETER_MODE_LIST = {COM_POSITION_X:     [BALANCE],
                       COM_POSITION_Y:     [BALANCE],
                       COM_POSITION_Z:     [BALANCE],
                       BASE_ORIENTATION_X: [BALANCE],
                       BASE_ORIENTATION_Y: [BALANCE],
                       BASE_ORIENTATION_Z: [BALANCE],
                       COM_VELOCITY_X:     [WALK],
                       COM_VELOCITY_Y:     [WALK],
                       BASE_YAW_RATE:      [WALK],
                       FOOT_YAW_RIGHT:     [WALK],
                       FOOT_YAW_LEFT:      [WALK],
                       FOOT_CLEARANCE_X:   [WALK],
                       FOOT_CLEARANCE_Y:   [WALK]
                       }

COM_VELOCITY_LIMIT        = {WALK: [+0.50, -0.30, +0.15, -0.15]} if HARDWARE else {WALK: [+1.00, -0.50, +0.20, -0.20]}  # [vx_max, vx_min, vy_max, vy_min]
COM_VELOCITY_COMPENSATION = {WALK: [+0.00, +0.00]}               if HARDWARE else {WALK: [+0.00, +0.00]}                # [vx_offset, vy_offset]