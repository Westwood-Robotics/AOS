#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "March 27, 2025"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Test battery thread
'''

import time
import Setting.robot_data as RDS
from Setting.Macros.locomotion_macros import *


if __name__ == '__main__':
    Themis = RDS.RobotDataManager()

    while True:
        Themis.update_battery_states()

        print("       Battery Status: {:+.0f} {:+.0f}".format(Themis.battery['battery_statuses'][0], Themis.battery['battery_statuses'][1]))
        print("         Error Status: {:+.0f} {:+.0f}".format(Themis.battery['error_statuses'][0], Themis.battery['error_statuses'][1]))
        print("      Battery Voltage: {:+.2f} {:+.2f}".format(Themis.battery['battery_voltages'][0], Themis.battery['battery_voltages'][1]))
        print("         Cell Voltage: {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f}".format(Themis.battery['cell_voltages'][0, 0], Themis.battery['cell_voltages'][0, 1], Themis.battery['cell_voltages'][0, 2], Themis.battery['cell_voltages'][0, 3], Themis.battery['cell_voltages'][0, 4], Themis.battery['cell_voltages'][0, 5], Themis.battery['cell_voltages'][0, 6], Themis.battery['cell_voltages'][0, 7]))
        print("                       {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f} {:+.2f}".format(Themis.battery['cell_voltages'][1, 0], Themis.battery['cell_voltages'][1, 1], Themis.battery['cell_voltages'][1, 2], Themis.battery['cell_voltages'][1, 3], Themis.battery['cell_voltages'][1, 4], Themis.battery['cell_voltages'][1, 5], Themis.battery['cell_voltages'][1, 6], Themis.battery['cell_voltages'][1, 7]))
        print("           DSG Status: {:+.0f} {:+.0f}".format(Themis.battery['dsg_statuses'][0], Themis.battery['dsg_statuses'][1]))
        print("           CHG Status: {:+.0f} {:+.0f}".format(Themis.battery['chg_statuses'][0], Themis.battery['chg_statuses'][1]))
        print("Max Discharge Current: {:+.2f} {:+.2f}".format(Themis.battery['max_discharge_currents'][0], Themis.battery['max_discharge_currents'][1]))
        print("   Max Charge Current: {:+.2f} {:+.2f}".format(Themis.battery['max_charge_currents'][0], Themis.battery['max_charge_currents'][1]))
        print("      Present Current: {:+.2f} {:+.2f}".format(Themis.battery['present_currents'][0], Themis.battery['present_currents'][1]))
        print("          Temperature: {:+.2f} {:+.2f}".format(Themis.battery['temperatures'][0], Themis.battery['temperatures'][1]))
        print()
        for _ in range(12):
            print("\033[1A", end="\x1b[2K")

        time.sleep(0.01)