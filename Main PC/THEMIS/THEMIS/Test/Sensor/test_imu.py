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
Test IMU
'''

import time
import Library.MATH_FUNCTION.math_function as MF
from Library.THEMIS_SENSE.IMU import Manager as imu_sense_manager


if __name__ == '__main__':
    try:
        ism = imu_sense_manager.IMUSenseManager(port='/dev/ttyACM0', baudrate=115200)
    except:
        ism = imu_sense_manager.IMUSenseManager(port='/dev/ttyACM1', baudrate=115200)

    t0, t1, t2 = 0, 0, 0
    f1, f2 = 0, 0
    try:
        while True:
            t0 = time.perf_counter()
            if ism.get_data():
                t1 = t2
                t2 = time.perf_counter()
                f1 = 1. / (t2 - t0)
                f2 = 1. / (t2 - t1)

                rotation_vector = MF.logvee(ism.base_rotation_matrix)
                
                print("   Acceleration: {:+.2f} {:+.2f} {:+.2f}".format(ism.base_acceleration[0], ism.base_acceleration[1], ism.base_acceleration[2]))
                print("   Angular Rate: {:+.2f} {:+.2f} {:+.2f}".format(ism.base_angular_rate[0], ism.base_angular_rate[1], ism.base_angular_rate[2]))
                print("   Euler Angles: {:+.2f} {:+.2f} {:+.2f}".format(ism.imu_euler_angles[0], ism.imu_euler_angles[1], ism.imu_euler_angles[2]))
                print("Rotation Vector: {:+.2f} {:+.2f} {:+.2f}".format(rotation_vector[0], rotation_vector[1], rotation_vector[2]))
                print("    Orientation: {:+.2f} {:+.2f} {:+.2f}".format(ism.base_rotation_matrix[0, 0], ism.base_rotation_matrix[0, 1], ism.base_rotation_matrix[0, 2]))
                print("                 {:+.2f} {:+.2f} {:+.2f}".format(ism.base_rotation_matrix[1, 0], ism.base_rotation_matrix[1, 1], ism.base_rotation_matrix[1, 2]))
                print("                 {:+.2f} {:+.2f} {:+.2f}".format(ism.base_rotation_matrix[2, 0], ism.base_rotation_matrix[2, 1], ism.base_rotation_matrix[2, 2]))
                print("    Read Frequency:  {:+.2f} [Hz]".format(f1))
                print("New Data Frequency:  {:+.2f} [Hz]".format(f2))
                print()
                for _ in range(10):
                    print("\033[1A", end="\x1b[2K")
                    
                # time.sleep(0.01)
    except:
        ism.disconnect()