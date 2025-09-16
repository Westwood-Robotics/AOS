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
Test estimation thread
'''

import time
import Setting.robot_data as RDS
import Library.MATH_FUNCTION.math_function as MF
from Setting.Macros.locomotion_macros import *


if __name__ == '__main__':
    Themis = RDS.RobotDataManager()

    while True:
        Themis.update_base_states()
        Themis.update_com_states()
        Themis.update_right_ankle_states()
        Themis.update_left_ankle_states()
        Themis.update_right_foot_states()
        Themis.update_left_foot_states()
        Themis.update_right_hand_states()
        Themis.update_left_hand_states()
        Themis.update_head_camera_states()

        tfr = MF.logvee(Themis.R_wf[RIGHT])
        tfl = MF.logvee(Themis.R_wf[LEFT])
        thr = MF.logvee(Themis.R_wh[RIGHT])
        thl = MF.logvee(Themis.R_wh[LEFT])
        tc  = MF.logvee(Themis.R_wc)

        print("          Base Position: {:+.2f} {:+.2f} {:+.2f}".format(Themis.p_wb[0], Themis.p_wb[1], Themis.p_wb[2]))
        print("          Base Velocity: {:+.2f} {:+.2f} {:+.2f}".format(Themis.v_wb[0], Themis.v_wb[1], Themis.v_wb[2]))
        print("           CoM Position: {:+.2f} {:+.2f} {:+.2f}".format(Themis.p_wg[0], Themis.p_wg[1], Themis.p_wg[2]))
        print("           CoM Velocity: {:+.2f} {:+.2f} {:+.2f}".format(Themis.v_wg[0], Themis.v_wg[1], Themis.v_wg[2]))
        print("       Base Orientation: {:+.2f} {:+.2f} {:+.2f}".format(Themis.R_wb[0, 0], Themis.R_wb[0, 1], Themis.R_wb[0, 2]))
        print("                         {:+.2f} {:+.2f} {:+.2f}".format(Themis.R_wb[1, 0], Themis.R_wb[1, 1], Themis.R_wb[1, 2]))
        print("                         {:+.2f} {:+.2f} {:+.2f}".format(Themis.R_wb[2, 0], Themis.R_wb[2, 1], Themis.R_wb[2, 2]))
        print()
        print("   Right Ankle Position: {:+.2f} {:+.2f} {:+.2f}".format(Themis.p_wa[RIGHT][0], Themis.p_wa[RIGHT][1], Themis.p_wa[RIGHT][2]))
        print("    Left Ankle Position: {:+.2f} {:+.2f} {:+.2f}".format(Themis.p_wa[LEFT][0],  Themis.p_wa[LEFT][1],  Themis.p_wa[LEFT][2]))
        print(" Right Foot Orientation: {:+.2f} {:+.2f} {:+.2f}".format(tfr[0], tfr[1], tfr[2]))
        print("  Left Foot Orientation: {:+.2f} {:+.2f} {:+.2f}".format(tfl[0], tfl[1], tfl[2]))
        print()
        print("    Right Hand Position: {:+.2f} {:+.2f} {:+.2f}".format(Themis.p_wh[RIGHT][0], Themis.p_wh[RIGHT][1], Themis.p_wh[RIGHT][2]))
        print("     Left Hand Position: {:+.2f} {:+.2f} {:+.2f}".format(Themis.p_wh[LEFT][0],  Themis.p_wh[LEFT][1],  Themis.p_wh[LEFT][2]))
        print(" Right Hand Orientation: {:+.2f} {:+.2f} {:+.2f}".format(thr[0], thr[1], thr[2]))
        print("  Left Hand Orientation: {:+.2f} {:+.2f} {:+.2f}".format(thl[0], thl[1], thl[2]))
        print()
        print("   Head Camera Position: {:+.2f} {:+.2f} {:+.2f}".format(Themis.p_wc[0], Themis.p_wc[1], Themis.p_wc[2]))
        print("Head camera Orientation: {:+.2f} {:+.2f} {:+.2f}".format(tc[0], tc[1], tc[2]))
        for _ in range(20):
            print("\033[1A", end="\x1b[2K")

        time.sleep(0.01)