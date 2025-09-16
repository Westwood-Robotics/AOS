#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 18, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Play Configuration
'''

SIMULATION = True   # if in simulation or not
GAMEPAD    = False  # if using gamepad or not
ESTIMATION = False  # if using estimator or not (only for simulation)

HARDWARE = not SIMULATION
if HARDWARE:
    ESTIMATION = True

if __name__ == '__main__':
    print(SIMULATION, GAMEPAD, ESTIMATION)