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
Script for bias calibration of IMU (DO NOT MOVE IMU IN CALIBRATION!!!)
'''

import time
from Library.THEMIS_SENSE.IMU.MSCL import mscl
from Library.THEMIS_SENSE.IMU import Manager as imu_sense_manager


if __name__ == '__main__':
    try:
        ism = imu_sense_manager.IMUSenseManager(port='/dev/ttyACM0', baudrate=115200)
    except:
        ism = imu_sense_manager.IMUSenseManager(port='/dev/ttyACM1', baudrate=115200)

    t = 30  # sample time [sec]
        
    packet = ism.node.getDataPackets(1000)
    print()

    omega_bias = ism.node.getGyroBias()
    print("SAVED GYROSCOPE BIAS: {:+.8f} {:+.8f} {:+.8f}".format(omega_bias.x(), omega_bias.y(), omega_bias.z()))

    ism.node.captureGyroBias(t * 1000)
    omega_bias = ism.node.getGyroBias()

    print("IMU CALIBRATION DONE!")
    print("NEW GYROSCOPE BIAS: {:+.8f} {:+.8f} {:+.8f}".format(omega_bias.x(), omega_bias.y(), omega_bias.z()))

    confirm = input("Save bias? (y/n) ")
    if confirm != 'y':
        ism.disconnect()
        exit()

    ism.node.setGyroBias(omega_bias)
    ism.node.saveSettingsAsStartup()

    print("GYROSCOPE BIAS SAVED.")

    ism.disconnect()