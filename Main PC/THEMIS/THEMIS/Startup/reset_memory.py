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

import numpy as np
try:
    import Startup.memory_manager as MM
except:
    import os
    import time
    os.system("python3 -m Startup.memory_manager")
    time.sleep(1)
    import Startup.memory_manager as MM


if __name__ == '__main__':
    MEMORY_SEGMENT_RESET_LIST = MM.MEMORY_SEGMENT_LIST.copy()
    MEMORY_SEGMENT_RESET_LIST.remove(MM.THREAD_STATE)
    
    for mem_seg in MEMORY_SEGMENT_RESET_LIST:
        mem_seg.set(opt='default')

    MM.THREAD_STATE.set({'simulation':     np.zeros(1),
                         'bear_right_leg': np.zeros(1),
                         'bear_left_leg':  np.zeros(1),
                         'bear_right_arm': np.zeros(1),
                         'bear_left_arm':  np.zeros(1),
                         'bear_head':      np.zeros(1),
                         'dxl_right_hand': np.zeros(1),
                         'dxl_left_hand':  np.zeros(1),
                         'sense':          np.zeros(1),
                         'estimation':     np.zeros(1),
                         'low_level':      np.zeros(1),
                         'high_level':     np.zeros(1),
                         'top_level':      np.zeros(1)}, opt='update')