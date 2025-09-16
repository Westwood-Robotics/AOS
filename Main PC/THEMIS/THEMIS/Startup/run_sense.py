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
Script for communication with IMU and pico
'''

import numpy as np
import Setting.robot_data as RDS
import Startup.memory_manager as MM
import Library.MATH_FUNCTION.math_function as MF
from termcolor import cprint
from Setting.Macros.constant_macros import *
from Library.THEMIS_SENSE.IMU  import Manager as imu_sense_manager


def main_loop():
    Themis = RDS.RobotDataManager()

    loop_freq     = 2000
    loop_duration = 1. / loop_freq

    check_thread_frequency = 10
    check_thread_duration  = 1. / check_thread_frequency

    filtered_accel = np.array([0., 0., GRAVITY_ACCEL])
    filtered_omega = np.zeros(3)

    print("====== THEMIS Sense Communication Thread is running at", loop_freq, "Hz... ======")

    last_check_thread_time = Themis.get_time()

    error_threshold = 10
    error_count = 0
    thread_check = False

    t0 = Themis.get_time()
    while True:
        loop_start_time = Themis.get_time()

        # check threading issue
        last_check_thread_elapse = loop_start_time - last_check_thread_time
        if last_check_thread_elapse > check_thread_duration:
            last_check_thread_time = loop_start_time
            if Themis.thread_issue():
                Themis.stop_threading()

        if not thread_check and loop_start_time - t0 > 1:
            cprint("THREAD IS DOING GREAT!", 'green')
            MM.THREAD_STATE.set({'sense': np.array([1])}, opt='update')  # thread is running
            thread_check = True

        if ism.get_data():
            data_error = False
            try:
                for idx in range(3):
                    if np.abs(ism.base_acceleration[idx]) > 1000 or np.abs(ism.imu_angular_rate[idx]) > 1000 or MF.norm(ism.base_rotation_matrix[:, idx]) > 1000:
                        data_error = True
                        error_count += 1
                        break
            except:
                data_error = True
                error_count += 1

            if not data_error:
                error_count = 0
                for idx in range(3):
                    filtered_accel[idx] = MF.exp_filter(filtered_accel[idx], ism.base_acceleration[idx], 0.10)
                    filtered_omega[idx] = MF.exp_filter(filtered_omega[idx], ism.base_angular_rate[idx], 0.10)
                sense_data = {'imu_acceleration':    filtered_accel,
                              'imu_angular_rate':    filtered_omega,
                              'imu_rotation_matrix': ism.base_rotation_matrix
                             }
                MM.SENSE_STATE.set(sense_data)
        else:
            error_count += error_threshold / loop_freq / 0.1  # error if no data in 0.1 seconds

        if error_count >= error_threshold:
            cprint("IMU ERROR! RESETTING!", 'yellow')
            ism.idle()
            error_count = 0

        # check time to ensure the thread stays at a consistent running loop
        loop_elapsed_time = Themis.get_time() - loop_start_time
        if loop_elapsed_time > loop_duration * 1.5:
            cprint("Delayed " + str(1e3 * (loop_elapsed_time - loop_duration))[0:5] + " ms", 'yellow')
        elif loop_elapsed_time < loop_duration:
            while Themis.get_time() - loop_start_time < loop_duration:
                pass


if __name__ == '__main__':
    # setup
    try:
        ism = imu_sense_manager.IMUSenseManager(port='/dev/ttyACM0', baudrate=IMU_BAUDRATE)
    except:
        ism = imu_sense_manager.IMUSenseManager(port='/dev/ttyACM1', baudrate=IMU_BAUDRATE)

    # run
    try:
        main_loop()
    except RDS.THREAD_STOP:
        cprint("THREAD STOPPED PEACEFULLY!", 'light_grey')
        MM.THREAD_STATE.set({'sense': np.array([0])}, opt='update')  # thread is stopped
    except (Exception, KeyboardInterrupt) as error:
        print(error)
        cprint("THREAD IN ERROR! TERMINATE NOW!", 'red')
        MM.THREAD_STATE.set({'sense': np.array([2])}, opt='update')  # thread in error
    finally:
        ism.disconnect()