#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 10, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Reset shared memory to default
'''

import Test.Memory.memory_manager as mm


if __name__ == '__main__':
    mm.TIME_STATE.set(opt='default')
    mm.JOINT_STATE.set(opt='default')