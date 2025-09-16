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
Test communication with DXL actuators
'''

import time
from termcolor import cprint
from Setting.Macros.dxl_macros import *
from Library.DXL_ACTUATOR import Manager as dxl_actuator_manager


if __name__ == '__main__':
    chain = input("Test right hand (rh) or left hand (lh) chain? ")
    if chain == 'rh':
        dam = dxl_actuator_manager.DXLActuatorManager(port=HAND_R_DXL_PORT, baudrate=DXL_BAUDRATE)
        dxl_list = HAND_R_DXL_ID_LIST
    elif chain == 'lh':
        dam = dxl_actuator_manager.DXLActuatorManager(port=HAND_L_DXL_PORT, baudrate=DXL_BAUDRATE)
        dxl_list = HAND_L_DXL_ID_LIST
    else:
        cprint("ERROR INPUT!", 'red')
        exit()

    # return delay time
    # error = dam.write(dxl_list, [DXL_REGISTER_RETURN_DELAY_TIME], [[DXL[DXL_RETURN_DELAY_TIME][dxl]] for dxl in dxl_list])
    # print(error)

    # present velocity, position
    gsr_present_velocity_position = dam.sync_read_setup(dxl_list, [DXL_REGISTER_PRESENT_VELOCITY, DXL_REGISTER_PRESENT_POSITION])

    while True:
        loop_start_time = time.time()
        data = dam.sync_read(gsr_present_velocity_position)
        # data, error = dam.read(dxl_list, [DXL_REGISTER_PRESENT_VELOCITY, DXL_REGISTER_PRESENT_POSITION])
        print(data)
        # time.sleep(0.2)
        print(1. / (time.time() - loop_start_time))