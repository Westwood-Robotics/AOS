#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "April 22, 2024"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Test gamepad thread
'''

import time
import Setting.robot_data as RDS


if __name__ == '__main__':
    Themis = RDS.RobotDataManager()
    
    while True:
        Themis.update_gamepad_states()

        for key in list(Themis.gamepad.keys()):
            void_msg = " " * (4 - len(key))
            if key in ['LX', 'LY', 'RX', 'RY', 'LSZ', 'RSZ']:
                print(void_msg + key + ": {:+.2f}".format(Themis.gamepad[key]))
            else:
                print(void_msg + key + ": {:+.0f}".format(Themis.gamepad[key]))

        print()
        for _ in range(len(list(Themis.gamepad)) + 1):
            print("\033[1A", end="\x1b[2K")

        time.sleep(0.01)