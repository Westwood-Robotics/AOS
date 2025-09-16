#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "August 30, 2024"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Script for monitoring gamepad data
'''

import time
from Library.THEMIS_GAMEPAD import Manager as gamepad_manager


if __name__ == '__main__':
    gm = gamepad_manager.GamepadManager()

    while True:
        gm.check_connection()
        if gm.is_connected:
            gm.read_data()
            for key in list(gm.button.keys()):
                print(key + ': {:.0f}'.format(gm.button[key]))
            print()
            for key in list(gm.axis.keys()):
                print(key + ': {:.2f}'.format(gm.axis[key]))
            for _ in range(len(list(gm.button.keys()) + list(gm.axis.keys())) + 1):
                print('\033[1A', end='\x1b[2K')
            time.sleep(0.01)
        else:
            gm.reconnect()