#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "March 3, 2025"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Test battery
'''

import time
from Setting.Macros.constant_macros import *
from Library.THEMIS_BATTERY import Manager as battery_manager


if __name__ == '__main__':
    bsm = battery_manager.BatteryManager(port=BATTERY_PORT, baudrate=BATTERY_BAUDRATE)

    t0, t1, t2 = 0, 0, 0
    f1, f2 = 0, 0
    try:
        while True:
            t0 = time.perf_counter()
            if bsm.get_data() and bsm.data4pico[2] == 1:
                t1 = t2
                t2 = time.perf_counter()
                f1 = 1. / (t2 - t0)
                f2 = 1. / (t2 - t1)

            print("       Battery Status: {:+.0f} {:+.0f}".format(bsm.battery_status[0], bsm.battery_status[1]))
            print("         Error Status: {:+.0f} {:+.0f}".format(bsm.error[0], bsm.error[1]))
            print("      Battery Voltage: {:+.2f} {:+.2f}".format(bsm.battery_voltage[0], bsm.battery_voltage[1]))
            print("         Cell Voltage: {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f}".format(bsm.cell_voltage[0, 0], bsm.cell_voltage[0, 1], bsm.cell_voltage[0, 2], bsm.cell_voltage[0, 3], bsm.cell_voltage[0, 4], bsm.cell_voltage[0, 5], bsm.cell_voltage[0, 6], bsm.cell_voltage[0, 7]))
            print("                       {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f}".format(bsm.cell_voltage[1, 0], bsm.cell_voltage[1, 1], bsm.cell_voltage[1, 2], bsm.cell_voltage[1, 3], bsm.cell_voltage[1, 4], bsm.cell_voltage[1, 5], bsm.cell_voltage[1, 6], bsm.cell_voltage[1, 7]))
            print("           DSG Status: {:+.0f} {:+.0f}".format(bsm.dsg_status[0], bsm.dsg_status[1]))
            print("           CHG Status: {:+.0f} {:+.0f}".format(bsm.chg_status[0], bsm.chg_status[1]))
            print("Max Discharge Current: {:+.2f} {:+.2f}".format(bsm.max_discharge_current[0], bsm.max_discharge_current[1]))
            print("   Max Charge Current: {:+.2f} {:+.2f}".format(bsm.max_charge_current[0], bsm.max_charge_current[1]))
            print("      Present Current: {:+.2f} {:+.2f}".format(bsm.present_current[0], bsm.present_current[1]))
            print("          Temperature: {:+.2f} {:+.2f}".format(bsm.temperature[0], bsm.temperature[1]))
            print()
            print("    Read Frequency: {:+.2f} [Hz]".format(f1))
            print("New Data Frequency: {:+.2f} [Hz]".format(f2))
            print()
            for _ in range(15):
                print("\033[1A", end="\x1b[2K")
                    
            time.sleep(0.1)
    except:
        bsm.close_port()