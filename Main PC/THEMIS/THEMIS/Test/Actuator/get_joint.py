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
Get joint states
'''

import time
import Setting.robot_data as RDS
from termcolor import cprint
from Setting.Macros.dxl_macros import *
from Setting.Macros.bear_macros import *
from Setting.Macros.model_macros import *


if __name__ == '__main__':
    Themis = RDS.RobotDataManager()

    print("====== THEMIS Joint State Info ======")
    option = input("Get joint position or velocity or torque or temperature or voltage info? ")

    if option == 'position':
        val1 = Themis.joint_position
        val2 = ROBOT_JOINT_ID_LIST
        val2.remove(JOINT_BASE)
    elif option == 'velocity':
        val1 = Themis.joint_velocity
        val2 = ROBOT_JOINT_ID_LIST
        val2.remove(JOINT_BASE)
    elif option == 'torque':
        val1 = Themis.joint_torque
        val2 = ROBOT_JOINT_ID_LIST
        val2.remove(JOINT_BASE)
    elif option == 'temperature':
        val1 = Themis.bear_temperature
        val2 = ROBOT_BEAR_ID_LIST
        val3 = Themis.dxl_temperature
        val4 = ROBOT_DXL_ID_LIST
    elif option == 'voltage':
        val1 = Themis.bear_voltage
        val2 = ROBOT_BEAR_ID_LIST
        val3 = Themis.dxl_voltage
        val4 = ROBOT_DXL_ID_LIST
    else:
        cprint("ERROR INPUT!", 'red')
        exit()
    
    while True:
        Themis.update_joint_states(chains=ROBOT_CHAIN_LIST)
        
        print("      Head: {:+.2f} / {:+.2f}".format(val1[val2[26]],
                                                     val1[val2[27]]))
        print(" Right Arm: {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f}".format(val1[val2[12]],
                                                                                                       val1[val2[13]],
                                                                                                       val1[val2[14]],
                                                                                                       val1[val2[15]],
                                                                                                       val1[val2[16]],
                                                                                                       val1[val2[17]],
                                                                                                       val1[val2[18]]))
        print("  Left Arm: {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f}".format(val1[val2[19]],
                                                                                                       val1[val2[20]],
                                                                                                       val1[val2[21]],
                                                                                                       val1[val2[22]],
                                                                                                       val1[val2[23]],
                                                                                                       val1[val2[24]],
                                                                                                       val1[val2[25]]))
        print(" Right Leg: {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f}".format(val1[val2[0]],
                                                                                             val1[val2[1]],
                                                                                             val1[val2[2]],
                                                                                             val1[val2[3]],
                                                                                             val1[val2[4]],
                                                                                             val1[val2[5]]))
        print("  Left Leg: {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f}".format(val1[val2[6]],
                                                                                             val1[val2[7]],
                                                                                             val1[val2[8]],
                                                                                             val1[val2[9]],
                                                                                             val1[val2[10]],
                                                                                             val1[val2[11]]))
        if option in ['temperature', 'voltage']:
            print("Right Hand: {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f}".format(val3[val4[0]],
                                                                                                           val3[val4[1]],
                                                                                                           val3[val4[2]],
                                                                                                           val3[val4[3]],
                                                                                                           val3[val4[4]],
                                                                                                           val3[val4[5]],
                                                                                                           val3[val4[6]]))
            print(" Left Hand: {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f}".format(val3[val4[7]],
                                                                                                           val3[val4[8]],
                                                                                                           val3[val4[9]],
                                                                                                           val3[val4[10]],
                                                                                                           val3[val4[11]],
                                                                                                           val3[val4[12]],
                                                                                                           val3[val4[13]]))
        else:
            print("Right Hand: {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f}".format(val1[val2[28]],
                                                                                                           val1[val2[29]],
                                                                                                           val1[val2[30]],
                                                                                                           val1[val2[31]],
                                                                                                           val1[val2[32]],
                                                                                                           val1[val2[33]],
                                                                                                           val1[val2[34]]))
            print(" Left Hand: {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f} / {:+.2f}".format(val1[val2[35]],
                                                                                                           val1[val2[36]],
                                                                                                           val1[val2[37]],
                                                                                                           val1[val2[38]],
                                                                                                           val1[val2[39]],
                                                                                                           val1[val2[40]],
                                                                                                           val1[val2[41]]))
        
        print()
        for _ in range(8):
            print("\033[1A", end="\x1b[2K")

        time.sleep(0.01)