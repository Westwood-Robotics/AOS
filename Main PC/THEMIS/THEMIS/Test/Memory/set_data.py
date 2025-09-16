#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 5, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Set data to shared memory
'''

import numpy as np
import Test.Memory.memory_manager as mm


if __name__ == '__main__':
    data = {'joint_positions':  np.array([+0.1, +0.2]),
            'joint_velocities': np.array([+0.3, +0.4])}
    mm.JOINT_STATE.set(data)

    data = {'joint_positions':  np.array([-0.1, -0.2])}
    mm.JOINT_STATE.set(data, opt='remain')