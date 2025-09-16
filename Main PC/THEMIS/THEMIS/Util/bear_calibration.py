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
Script for homing calibration of BEAR actuators
'''

import time
from Setting.Macros.bear_macros import *
from Library.BEAR_ACTUATOR import Manager as bear_actuator_manager


def home_bear(bam, bear_id):
    print("Homing BEAR ID: %02d" % bear_id)

    if bear_id == BEAR_HIP_PITCH_R:
        desired_position = +75 * DEG2RAD
    elif bear_id == BEAR_HIP_PITCH_L:
        desired_position = -75 * DEG2RAD
    elif bear_id == BEAR_KNEE_R:
        desired_position = -0.4073 * DEG2RAD
    elif bear_id == BEAR_KNEE_L:
        desired_position = +0.4073 * DEG2RAD
    elif bear_id in [BEAR_ANKLE_IN_R, BEAR_ANKLE_OUT_L]:
        desired_position = -0.9846 + 0.07 * 1
    elif bear_id in [BEAR_ANKLE_IN_L, BEAR_ANKLE_OUT_R]:
        desired_position = +0.9846 - 0.07 * 1
    elif bear_id == BEAR_HEAD_PITCH:
        desired_position = +61 * DEG2RAD
    else:
        desired_position = 0

    present_position = 0
    sample_num = 10
    for _ in range(sample_num):
        present_position += bam.get_present_position([bear_id])[0] / sample_num
        time.sleep(0.01)

    present_offset = bam.get_homing_offset([bear_id])[0]

    new_offset = desired_position + present_offset - present_position
    if new_offset > 2 ** 17:
        new_offset -= 2 ** 18
    if new_offset < -2 ** 17:
        new_offset += 2 ** 18

    diff_offset = new_offset - present_offset

    print("Present Offset: {:.2f}".format(present_offset))
    print("    New Offset: {:.2f}".format(new_offset))
    print("    Difference: {:.2f}".format(diff_offset))

    new_position = present_position + diff_offset

    print("New Position w.r.t. New Offset: {:.2f}".format(new_position))

    confirm = input("Set new offset? (y/n)")
    if confirm == 'y':
        bam.set_homing_offset([bear_id], [new_offset])
        time.sleep(0.2)
        offset_reading = bam.get_homing_offset([bear_id])[0]
        if abs(offset_reading - new_offset) > 0.01:
            print("Failed to write homing offset!!!")
        else:
            bam.save_config([bear_id])
            print("Homing succeeded.")
    else:
        print("Homing canceled.")
    print("")


if __name__ == '__main__':
    print("====== THEMIS BEAR Homing Calibration ======")
    chain = input("Calibrate right leg (rl) or left leg (ll) or right arm (ra) or left arm (la) or head (h) chain? ")
    if chain == 'rl':
        bam = bear_actuator_manager.BearActuatorManager(port=LEG_R_BEAR_PORT,  baudrate=BEAR_BAUDRATE)
        for bear_id in LEG_R_BEAR_ID_LIST:
            home_bear(bam, bear_id)
    elif chain == 'll':
        bam = bear_actuator_manager.BearActuatorManager(port=LEG_L_BEAR_PORT,   baudrate=BEAR_BAUDRATE)
        for bear_id in LEG_L_BEAR_ID_LIST:
            home_bear(bam, bear_id)
    elif chain == 'ra':
        bam = bear_actuator_manager.BearActuatorManager(port=ARM_R_BEAR_PORT, baudrate=BEAR_BAUDRATE)
        for bear_id in ARM_R_BEAR_ID_LIST:
            home_bear(bam, bear_id)
    elif chain == 'la':
        bam = bear_actuator_manager.BearActuatorManager(port=ARM_L_BEAR_PORT, baudrate=BEAR_BAUDRATE)
        for bear_id in ARM_L_BEAR_ID_LIST:
            home_bear(bam, bear_id)
    elif chain == 'h':
        bam = bear_actuator_manager.BearActuatorManager(port=HEAD_BEAR_PORT, baudrate=BEAR_BAUDRATE)
        for bear_id in HEAD_BEAR_ID_LIST:
            home_bear(bam, bear_id)
    else:
        print("Error input!")