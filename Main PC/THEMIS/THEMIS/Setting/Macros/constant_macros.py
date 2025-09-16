#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 3, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Script that holds useful constant macros
'''

import numpy as np


# ==================================================
# ==================================================
# π
# ==================================================
# ==================================================
PI   = np.pi
PI_2 = PI / 2.
PI_3 = PI / 3.
PI_4 = PI / 4.

PI2 = PI * 2.

DEG2RAD = PI / 180.
RAD2DEG = 180. / PI


# ==================================================
# ==================================================
# GRAVITY
# ==================================================
# ==================================================
GRAVITY_ACCEL = 9.81


# ==================================================
# ==================================================
# SERIAL PORT
# ==================================================
# ==================================================
try:
    with open('/home/themis/THEMIS/THEMIS_SERIAL_PORT') as f:
        lines = f.readlines()
    LEG_R_BEAR_PORT = lines[0][0:-1]
    LEG_L_BEAR_PORT = lines[1][0:-1]
    ARM_R_BEAR_PORT = lines[2][0:-1]
    ARM_L_BEAR_PORT = lines[3][0:-1]
    HEAD_BEAR_PORT  = lines[4][0:-1]
    HAND_R_DXL_PORT = lines[5][0:-1]
    HAND_L_DXL_PORT = lines[6][0:-1]
    IMU_PORT        = lines[7][0:-1]
    BATTERY_PORT    = lines[8][0:-1]
    PICO_PORT       = lines[9][0:-1]
    GAMEPAD_MAC     = lines[10][0:-1]
except:
    LEG_R_BEAR_PORT = '/dev/ttyUSB0'
    LEG_L_BEAR_PORT = '/dev/ttyUSB1'
    ARM_R_BEAR_PORT = '/dev/ttyUSB2'
    ARM_L_BEAR_PORT = '/dev/ttyUSB3'
    HEAD_BEAR_PORT  = '/dev/ttyUSB4'
    HAND_R_DXL_PORT = '/dev/ttyUSB5'
    HAND_L_DXL_PORT = '/dev/ttyUSB6'
    IMU_PORT        = '/dev/ttyACM0'
    BATTERY_PORT    = '/dev/ttyACM1'
    PICO_PORT       = '/dev/ttyACM2'
    GAMEPAD_MAC     = '20:43:A8:62:30:86'

BEAR_BAUDRATE    = 8000000
DXL_BAUDRATE     = 2000000
BATTERY_BAUDRATE = 115200
PICO_BAUDRATE    = 115200
IMU_BAUDRATE     = 115200
