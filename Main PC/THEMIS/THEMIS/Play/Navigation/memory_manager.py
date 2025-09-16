#!usr/bin/env python
__author__    = "Westwoodrobotics"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "April 9, 2025"
__update__    = "Aug 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Pre-generate the shared memory segments in advance
'''

import numpy as np
from Library.SHARED_MEMORY import Manager as shared_memory_manager
# Navigation Command
NAVIGATION_COMMAND = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='NAVIGATION_COMMAND', init=False)
NAVIGATION_COMMAND.add_block(name='horizontal_velocity', data=np.zeros(2))
NAVIGATION_COMMAND.add_block(name='yaw_rate',            data=np.zeros(1))
NAVIGATION_COMMAND.add_block(name='in_avaliable_region', data=np.zeros(1))
NAVIGATION_COMMAND.add_block(name='navigation_mode',     data=np.zeros(1))
NAVIGATION_COMMAND.add_block(name='head_yaw',            data=np.zeros(1))
NAVIGATION_COMMAND.add_block(name='head_pitch',          data=np.zeros(1))
NAVIGATION_COMMAND.add_block(name='navigation_status',   data=np.zeros(1))
NAVIGATION_COMMAND.add_block(name='navigation',   data=np.zeros(1))

def init():
    """
    Initialize if main
    """
    NAVIGATION_COMMAND.initialize = True

def connect():
    """
    Connect and create segment
    """
    NAVIGATION_COMMAND.connect_segment()

if __name__ == '__main__':
    init()
    connect()
else:
    connect()
