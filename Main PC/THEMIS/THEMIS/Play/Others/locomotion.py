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
THEMIS Locomotion API
ATTENTION: do not make changes to this file unless you know what you are doing :)
'''

import numpy as np
import Startup.memory_manager as MM


def set_locomotion_mode(m):
    """
    Set locomotion mode

    ARGUMENT:
    m (int) - standing (0), walking (1)
    """
    data = {'locomotion_mode': np.array([m])}
    MM.USER_COMMAND.set(data, opt='update')

def set_walking_velocity(v, w):
    """
    Set walking velocity

    ARGUMENT:
    v (1 x 2) - horizontal velocity
    w (float) - yaw rate
    """
    data = {'horizontal_velocity': np.array([v]),
            'yaw_rate':            np.array([w])}
    MM.USER_COMMAND.set(data, opt='update')

def set_base_orientation(roll, pitch, yaw):
    """
    Set base orientation

    ARGUMENT:
    roll  (float) - roll  angle [rad]
    pitch (float) - pitch angle [rad]
    yaw   (float) - yaw   angle [rad]
    """
    data = {'base_euler_angle_change': np.array([roll, pitch, yaw])}
    MM.USER_COMMAND.set(data, opt='update')

def set_com_position(x, y, z):
    """
    Set com position

    ARGUMENT:
    x (float) - x position [m]
    y (float) - y position [m]
    z (float) - z position [m]
    """
    data = {'com_position_change': np.array([x, y, z])}
    MM.USER_COMMAND.set(data, opt='update')