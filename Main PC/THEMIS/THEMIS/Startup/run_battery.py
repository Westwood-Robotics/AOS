#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "March 5, 2025"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Script for communication with battery
'''

import time
import numpy as np
import Setting.robot_data as RDS
import Startup.memory_manager as MM
from termcolor import cprint
from Setting.Macros.constant_macros import *
from Library.THEMIS_BATTERY import Manager as battery_manager


def main_loop():
    loop_freq     = 5
    loop_duration = 1. / loop_freq

    print("====== THEMIS Battery Communication Thread is running at", loop_freq, "Hz... ======")

    thread_check = False
    t0 = time.perf_counter()
    while True:
        loop_start_time = time.perf_counter()

        if not thread_check and loop_start_time - t0 > 1:
            cprint("THREAD IS DOING GREAT!", 'green')
            MM.THREAD_STATE.set({'battery': np.array([1])}, opt='update')  # thread is running
            thread_check = True

        if bsm.get_data():
            battery_data = {'battery_statuses':       bsm.battery_status,
                            'error_statuses':         bsm.error,
                            'battery_voltages':       bsm.battery_voltage,
                            'cell_voltages':          bsm.cell_voltage,
                            'dsg_statuses':           bsm.dsg_status,
                            'chg_statuses':           bsm.chg_status,
                            'max_discharge_currents': bsm.max_discharge_current,
                            'max_charge_currents':    bsm.max_charge_current,
                            'present_currents':       bsm.present_current,
                            'temperatures':           bsm.temperature}
            MM.BATTERY_STATE.set(battery_data)

        # check time to ensure the thread stays at a consistent running loop
        loop_elapsed_time = time.perf_counter() - loop_start_time
        if loop_elapsed_time > loop_duration * 1.5:
            cprint("Delayed " + str(1e3 * (loop_elapsed_time - loop_duration))[0:5] + " ms", 'yellow')
        elif loop_elapsed_time < loop_duration:
            while time.perf_counter() - loop_start_time < loop_duration:
                pass


if __name__ == '__main__':
    # setup
    bsm = battery_manager.BatteryManager(port=BATTERY_PORT, baudrate=BATTERY_BAUDRATE)

    # run
    try:
        main_loop()
    except RDS.THREAD_STOP:
        cprint("THREAD STOPPED PEACEFULLY!", 'light_grey')
        MM.THREAD_STATE.set({'battery': np.array([0])}, opt='update')  # thread is stopped
    except (Exception, KeyboardInterrupt) as error:
        print(error)
        cprint("THREAD IN ERROR! TERMINATE NOW!", 'red')
        MM.THREAD_STATE.set({'battery': np.array([2])}, opt='update')  # thread in error
    finally:
        bsm.close_port()