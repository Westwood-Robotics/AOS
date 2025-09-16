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
Test sense thread
'''

import time
import Setting.robot_data as RDS
import Library.MATH_FUNCTION.math_function as MF
from Setting.Macros.locomotion_macros import *


if __name__ == '__main__':
    Themis = RDS.RobotDataManager()

    while True:
        Themis.update_sense_states()

        rotation_vector = MF.logvee(Themis.imu_rotation_matrix)

        print("     Right Foot: {:+.0f} {:+.0f} {:+.0f}".format(Themis.pico_foot_contact_state[RIGHT][0], Themis.pico_foot_contact_state[RIGHT][1], Themis.pico_foot_contact_state[RIGHT][2]))
        print("      Left Foot: {:+.0f} {:+.0f} {:+.0f}".format(Themis.pico_foot_contact_state[LEFT][0],  Themis.pico_foot_contact_state[LEFT][1],  Themis.pico_foot_contact_state[LEFT][2]))
        print()
        print("   Acceleration: {:+.2f} {:+.2f} {:+.2f}".format(Themis.imu_acceleration[0], Themis.imu_acceleration[1], Themis.imu_acceleration[2]))
        print("   Angular Rate: {:+.2f} {:+.2f} {:+.2f}".format(Themis.imu_angular_rate[0], Themis.imu_angular_rate[1], Themis.imu_angular_rate[2]))
        print("Rotation Vector: {:+.2f} {:+.2f} {:+.2f}".format(rotation_vector[0], rotation_vector[1], rotation_vector[2]))
        print("    Orientation: {:+.2f} {:+.2f} {:+.2f}".format(Themis.imu_rotation_matrix[0, 0], Themis.imu_rotation_matrix[0, 1], Themis.imu_rotation_matrix[0, 2]))
        print("                 {:+.2f} {:+.2f} {:+.2f}".format(Themis.imu_rotation_matrix[1, 0], Themis.imu_rotation_matrix[1, 1], Themis.imu_rotation_matrix[1, 2]))
        print("                 {:+.2f} {:+.2f} {:+.2f}".format(Themis.imu_rotation_matrix[2, 0], Themis.imu_rotation_matrix[2, 1], Themis.imu_rotation_matrix[2, 2]))
        print()
        for _ in range(10):
            print("\033[1A", end="\x1b[2K")

        time.sleep(0.01)