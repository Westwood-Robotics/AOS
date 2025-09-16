#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "September 12, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Test communication with BEAR actuators
'''

import time
from termcolor import cprint
from Setting.Macros.bear_macros import *
from Library.BEAR_ACTUATOR import Manager as bear_actuator_manager


if __name__ == '__main__':
    chain = input("Test right leg (rl) or left leg (ll) or right arm (ra) or left arm (la) or head (h) chain? ")
    if chain == 'rl':
        bam = bear_actuator_manager.BearActuatorManager(port=LEG_R_BEAR_PORT, baudrate=BEAR_BAUDRATE)
        bear_list = LEG_R_BEAR_ID_LIST
    elif chain == 'll':
        bam = bear_actuator_manager.BearActuatorManager(port=LEG_L_BEAR_PORT, baudrate=BEAR_BAUDRATE)
        bear_list = LEG_L_BEAR_ID_LIST
    elif chain == 'ra':
        bam = bear_actuator_manager.BearActuatorManager(port=ARM_R_BEAR_PORT, baudrate=BEAR_BAUDRATE)
        bear_list = ARM_R_BEAR_ID_LIST
        # bear_list = [13]
    elif chain == 'la':
        bam = bear_actuator_manager.BearActuatorManager(port=ARM_L_BEAR_PORT, baudrate=BEAR_BAUDRATE)
        bear_list = ARM_L_BEAR_ID_LIST
    elif chain == 'h':
        bam = bear_actuator_manager.BearActuatorManager(port=HEAD_BEAR_PORT,  baudrate=BEAR_BAUDRATE)
        bear_list = HEAD_BEAR_ID_LIST
    else:
        cprint("ERROR INPUT!", 'red')
        exit()

    while True:
        loop_start_time = time.perf_counter()
        data = bam.bulk_read_stat(bear_list, ['present_position', 'present_velocity', 'present_iq', 'torque_enable'])
        # print(data)
        print(1. / (time.perf_counter() - loop_start_time))
