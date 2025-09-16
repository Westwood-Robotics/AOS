#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 29, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Script for communication with THEMIS gamepad
'''

# import os
import time
import Startup.memory_manager as MM
from Play.config import *
from Setting.Macros.constant_macros import *


def main_loop():
    # booted       = False
    # bear_status  = 0
    # thread_data  = MM.THREAD_STATE.get()
    gamepad_data = MM.GAMEPAD_STATE.get()
    # input_data   = MM.USER_COMMAND.get()
    # leg_data     = MM.RIGHT_LEG_JOINT_STATE.get()
    # leg_command  = MM.RIGHT_LEG_JOINT_COMMAND.get()
    # thread_error = False

    # send_data_frequency = 10
    # send_data_dt        = 1. / send_data_frequency
    # last_send_data_time = 0

    while True:
        gm.check_connection()
        if gm.is_connected:
            # set gamepad states to shared memory
            gamepad_data['connection'][0] = True
            if gm.read_data():
                for key in list(gm.button.keys()):
                    gamepad_data[key] = np.array([gm.button[key]])
                for key in list(gm.axis.keys()):
                    gamepad_data[key] = np.array([gm.axis[key]])
                MM.GAMEPAD_STATE.set(gamepad_data)

            # bootup/terminate
            # if HARDWARE and GAMEPAD and not DECK:
            #     if booted:
            #         if (gm.button['LZ'] and gm.button['RZ']) or thread_error:
            #             booted = False
            #             os.system("gnome-terminal -- '/home/themis/THEMIS/THEMIS/Play/terminate.sh'")
            #             print('THEMIS is terminated.')
            #         elif gm.button['ST'] and thread_data['top_level'][0] == 0:
            #             os.system("kill -18 $(pgrep bootup_gamepad)")
            #     else:
            #         if (gm.button['LZ'] and gm.button['ST']):
            #             booted = True
            #             # reset thread status
            #             for key in list(thread_data.keys()):
            #                 thread_data[key][0] = 0.0
            #             MM.THREAD_STATE.set(thread_data)
            #             thread_error = False
            #             os.system("gnome-terminal -- '/home/themis/THEMIS/THEMIS/Play/bootup_gamepad.sh'")
            #             print('')
            #             print('THEMIS is launched.')
            #             time.sleep(2)
            #         elif gm.button['LZ'] and gm.button['RZ'] and gm.button['LS'] and gm.button['RS']:
            #             if HARDWARE:
            #                 os.system("gnome-terminal -- '/home/themis/THEMIS/THEMIS/Util/poweroff.sh'")

            # # send status to gamepad
            # if time.time() - last_send_data_time >= send_data_dt:
            #     last_send_data_time = time.time()

            #     thread_data = MM.THREAD_STATE.get()
            #     for key in list(thread_data.keys()):
            #         if thread_data[key][0] == 2:
            #             thread_error = True
            #             break

            #     if thread_data['bear_right_leg'][0] == 0 and thread_data['bear_left_leg'][0] == 0:
            #         bear_status = 0
            #     if thread_data['bear_right_leg'][0] == 1 and thread_data['bear_left_leg'][0] == 1:
            #         bear_status = 1
            #     elif thread_data['bear_right_leg'][0] == 2 or thread_data['bear_left_leg'][0] == 2:
            #         bear_status = 2
            #         thread_error = True
            #     elif thread_data['bear_right_leg'][0] == 3 or thread_data['bear_left_leg'][0] == 3:
            #         bear_status = 3
            #         thread_error = True
            #     elif thread_data['bear_right_leg'][0] == 4 or thread_data['bear_left_leg'][0] == 4:
            #         bear_status = 4
            #         thread_error = True

            #     if booted:
            #         if thread_data['bear_right_leg'][0] == 1 and leg_command['bear_enable_statuses'][0] == 0:
            #             leg_command = MM.RIGHT_LEG_JOINT_COMMAND.get()
            #         if thread_data['top_level'][0] == 1:
            #             leg_data   = MM.RIGHT_LEG_JOINT_STATE.get()
            #             input_data = MM.USER_COMMAND.get()

            #     if SIMULATION:
            #         booted = 1
            #         bear_status = 1
            #         leg_command['bear_enable_statuses'][0] = 1
            #         if not ESTIMATION:
            #             thread_data['estimation'][0] = thread_data['simulation'][0]

            #     gm.send_data(            BOOT = booted,
            #                              BEAR = bear_status,
            #                        INITIALIZE = leg_command['bear_enable_statuses'][0],
            #                        ESTIMATION = thread_data['estimation'][0],
            #                         LOW_LEVEL = thread_data['low_level'][0],
            #                        HIGH_LEVEL = thread_data['high_level'][0],
            #                         TOP_LEVEL = thread_data['top_level'][0],
            #                   LOCOMOTION_MODE = input_data['locomotion_mode'][0],
            #                  BEAR_TEMPERATURE = np.max(leg_data['bear_temperatures']),
            #                      BEAR_VOLTAGE = np.min(leg_data['bear_voltages']))
                
            time.sleep(0.02)
        else:
            gamepad_data['connection'][0] = False
            for key in list(gm.button.keys()):
                gamepad_data[key] = np.array([0])
            for key in list(gm.axis.keys()):
                gamepad_data[key] = np.array([0])
            MM.GAMEPAD_STATE.set(gamepad_data)
            
            gm.reconnect()


if __name__ == '__main__':
    if GAMEPAD:
        from Library.THEMIS_GAMEPAD import Manager as gamepad_manager
        gm = gamepad_manager.GamepadManager()

        try:
            main_loop()
        except Exception as error:
            print(error)
        finally:
            gm.disconnect()
    else:
        print("GAMEPAD is not in use.")