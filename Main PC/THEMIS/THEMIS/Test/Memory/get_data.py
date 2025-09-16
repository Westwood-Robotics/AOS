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
Get data from shared memory
'''

import Test.Memory.memory_manager as mm
import Startup.memory_manager as MM


if __name__ == '__main__':
    data = mm.JOINT_STATE.get()
    print(data)

    data = MM.THREAD_STATE.get()
    print(data)