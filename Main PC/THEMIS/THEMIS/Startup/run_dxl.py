#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "February 27, 2025"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Script for communication with DXL actuators
'''

import time
import Setting.robot_data as RDS
import Startup.memory_manager as MM
import Library.MATH_FUNCTION.math_function as MF
import Library.ROBOT_MODEL.THEMIS_kinematics as KIN
from termcolor import cprint
from Setting.Macros.dxl_macros import *
from Setting.Macros.model_macros import *
from Setting.Macros.constant_macros import *
from Library.DXL_ACTUATOR.dynamixel_sdk import *
from Library.DXL_ACTUATOR import Manager as dxl_actuator_manager


class DXLControlManager(object):
    def __init__(self, chain=None):
        # THEMIS BEAR chain
        self.chain = chain

        # DXL serial port
        self.dxl_baudrate = DXL_BAUDRATE
        if self.chain == CHAIN_HAND_R:
            self.dxl_port = HAND_R_DXL_PORT
            self.dxl_list = HAND_R_DXL_ID_LIST
            self.read_write_frequency = 20
        elif self.chain == CHAIN_HAND_L:
            self.dxl_port = HAND_L_DXL_PORT
            self.dxl_list = HAND_L_DXL_ID_LIST
            self.read_write_frequency = 20
        # elif self.chain == CHAIN_HAND:
        #     self.dxl_port = HAND_R_DXL_PORT
        #     self.dxl_list = HAND_DXL_ID_LIST
        #     self.read_write_frequency = 20

        # DXL manager
        self.dam = dxl_actuator_manager.DXLActuatorManager(port=self.dxl_port, baudrate=self.dxl_baudrate)

        # DXL info
        self.dxl_num       = len(self.dxl_list)
        self.dxl_num_range = range(self.dxl_num)

        self.dxl_temperature_list = [20] * self.dxl_num
        self.dxl_voltage_list     = [ 5] * self.dxl_num

        self.dxl_operating_mode_list = [-99] * self.dxl_num
        self.dxl_enable_status_list  = [-99] * self.dxl_num

        self.dxl_proportional_gain_list = [0] * self.dxl_num
        self.dxl_derivative_gain_list   = [0] * self.dxl_num

        self.dxl_filtered_velocity_list = [0] * self.dxl_num
        self.dxl_filtered_position_list = [0] * self.dxl_num
        self.dxl_filtered_current_list  = [0] * self.dxl_num

        # self.communication_error_count     = 0
        # self.communication_error_threshold = 10

        # self.check_enable_status            = False
        self.update_temperature_and_voltage = False

        # initialization
        for dxl in self.dxl_list:
            self.dam.reboot([dxl])
        time.sleep(1)

        self.check_dxl()
        self.setup_group_sync()
        self.set_enable_status(values=[self.dam.enable_status['disable']]*self.dxl_num)
        self.setup_config()
        self.set_operating_mode(values=[self.dam.operating_mode['position']]*self.dxl_num)

    def check_dxl(self):
        """
        Ping all DXLs
        """
        # ping
        offline = [True] * self.dxl_num
        error   = [True] * self.dxl_num
        for idx, dxl in enumerate(self.dxl_list):
            trial = 0
            print("Pinging DXL %02d ..." % dxl)
            while offline[idx]:
                ping_info = self.dam.ping([dxl])
                if ping_info[0][1] == COMM_SUCCESS:
                    offline[idx] = False
                    print("Pinging DXL %02d Succeed." % dxl)
                    for jdx in range(3):
                        error_info, _ = self.dam.read([dxl], [DXL_REGISTER_HARDWARE_ERROR_STATUS])
                        if error_info[0][0] == 0:
                            error[idx] = False
                            if jdx > 0:
                                print("Clearing Error Succeed!")
                            break
                        else:
                            if jdx == 0:
                                print(error_info[0][0])
                                cprint("DXL %02d ERROR: " % dxl + self.dam.decode_error(error_info[0][0]) + ". Rebooting Now ...", 'red')
                            elif jdx == 2:
                                cprint("Clearing Error Failed!", 'red')
                                break
                            self.dam.reboot([dxl])
                            time.sleep(1)
                else:
                    trial += 1
                    if trial == 2:
                        cprint("Pinging DXL %02d Failed. Retrying ..." % dxl, 'yellow')
                    elif trial >= 6:
                        cprint("DXL %02d Offline!!!" % dxl, 'red')
                        break
                    time.sleep(0.5)
            print()
            time.sleep(0.1)
            
        if sum(offline) != 0 or sum(error) != 0:
            raise RDS.DXL_ERROR

    def setup_config(self):
        """
        Setup configuration registers
        """
        # return delay time
        error = self.dam.write(self.dxl_list, [DXL_REGISTER_RETURN_DELAY_TIME], [[DXL[DXL_RETURN_DELAY_TIME][dxl]] for dxl in self.dxl_list])

        # position pid
        error = self.dam.write(self.dxl_list, [DXL_REGISTER_POSITION_P_GAIN, DXL_REGISTER_POSITION_I_GAIN, DXL_REGISTER_POSITION_D_GAIN], [DXL[DXL_POSITION_PID][dxl] for dxl in self.dxl_list])
        
        # limit
        error = self.dam.write(self.dxl_list, [DXL_REGISTER_MIN_POSITION_LIMIT, DXL_REGISTER_MAX_POSITION_LIMIT], [DXL[DXL_POSITION_LIMIT][dxl] for dxl in self.dxl_list])
        error = self.dam.write(self.dxl_list, [DXL_REGISTER_CURRENT_LIMIT], [[DXL[DXL_CURRENT_LIMIT][dxl]] for dxl in self.dxl_list])
        error = self.dam.write(self.dxl_list, [DXL_REGISTER_TEMPERATURE_LIMIT], [[DXL[DXL_TEMPERATURE_LIMIT][dxl]] for dxl in self.dxl_list])

        print("Setting Config Succeed.", end='\n\n')

    def setup_group_sync(self):
        """
        Setup group sync
        """
        # operating mode
        self.gsw_operating_mode = self.dam.sync_write_setup(self.dxl_list, [DXL_REGISTER_OPERATING_MODE])
        self.gsr_operating_mode = self.dam.sync_read_setup(self.dxl_list, [DXL_REGISTER_OPERATING_MODE])

        # torque enable
        self.gsw_torque_enable = self.dam.sync_write_setup(self.dxl_list, [DXL_REGISTER_TORQUE_ENABLE])
        self.gsr_torque_enable = self.dam.sync_read_setup(self.dxl_list, [DXL_REGISTER_TORQUE_ENABLE])

        # position PID
        self.gsw_position_pid = self.dam.sync_write_setup(self.dxl_list, [DXL_REGISTER_POSITION_D_GAIN, DXL_REGISTER_POSITION_I_GAIN, DXL_REGISTER_POSITION_P_GAIN])

        # goal position
        self.gsw_goal_position = self.dam.sync_write_setup(self.dxl_list, [DXL_REGISTER_GOAL_POSITION])

        # present velocity, position
        self.gsr_present_velocity_position = self.dam.sync_read_setup(self.dxl_list, [DXL_REGISTER_PRESENT_VELOCITY, DXL_REGISTER_PRESENT_POSITION])

        # present voltage, temperature
        self.gsr_present_voltage_temperature = self.dam.sync_read_setup(self.dxl_list, [DXL_REGISTER_PRESENT_INPUT_VOLTAGE, DXL_REGISTER_PRESENT_TEMPERATURE])

        print("Setting Group Sync Succeed.", end='\n\n')

    def read_write_dxl(self, positions, p_gains=None, d_gains=None):
        """
        Get joint states and set joint commands
        """
        data_vp = self.dam.sync_read(self.gsr_present_velocity_position)

        if data_vp is not None:
            for idx in self.dxl_num_range:
                self.dxl_filtered_velocity_list[idx] = MF.exp_filter(self.dxl_filtered_velocity_list[idx], data_vp[idx][0], 0.0)
                self.dxl_filtered_position_list[idx] = MF.exp_filter(self.dxl_filtered_position_list[idx], data_vp[idx][1], 0.0)
                # self.dxl_filtered_current_list[idx]  = MF.exp_filter(self.dxl_filtered_current_list[idx],  info[idx][2], 0.1)

        if self.update_temperature_and_voltage:
            data_vt = self.dam.sync_read(self.gsr_present_voltage_temperature)
            if data_vt is not None:
                for idx, dxl in enumerate(self.dxl_list):
                    self.dxl_voltage_list[idx]     = data_vt[idx][0] * 0.1
                    self.dxl_temperature_list[idx] = data_vt[idx][1]
                    if self.dxl_temperature_list[idx] > 70:
                        cprint("DXL %02d HIGH TERPERATURE!!! " % dxl + "Detected Temperature: {:.2f}°C".format(self.dxl_temperature_list[idx]), 'yellow')
                    if self.dxl_voltage_list[idx] < 3:
                        cprint("DXL %02d LOW VOLTAGE!!! " % dxl + "Detected Voltage: {:.2f}V".format(self.dxl_voltage_list[idx]), 'yellow')
                self.update_temperature_and_voltage = False

        # update joint states
        if self.chain == CHAIN_HAND_R:
            right_hand_data = {'joint_positions':  KIN.motor2joint(0, +3, 0, np.array(self.dxl_filtered_position_list)),
                               'joint_velocities': KIN.motor2joint(0, +3, 1, np.array(self.dxl_filtered_velocity_list)),
                               #    'joint_torques':    KIN.motor2joint(0, +3, 2, np.array(self.dxl_filtered_current_list)),
                               'dxl_temperatures': np.array(self.dxl_temperature_list),
                               'dxl_voltages':     np.array(self.dxl_voltage_list)}
            MM.RIGHT_HAND_JOINT_STATE.set(right_hand_data)
            pos_data = KIN.joint2motor(0, +3, 0, positions)
        elif self.chain == CHAIN_HAND_L:
            left_hand_data = {'joint_positions':  KIN.motor2joint(0, -3, 0, np.array(self.dxl_filtered_position_list)),
                              'joint_velocities': KIN.motor2joint(0, -3, 1, np.array(self.dxl_filtered_velocity_list)),
                              #   'joint_torques':    KIN.motor2joint(0, +3, 2, np.array(self.dxl_filtered_current_list)),
                              'dxl_temperatures': np.array(self.dxl_temperature_list),
                              'dxl_voltages':     np.array(self.dxl_voltage_list)}
            MM.LEFT_HAND_JOINT_STATE.set(left_hand_data)
            pos_data = KIN.joint2motor(0, -3, 0, positions)
        # elif self.chain == CHAIN_HAND:
        #     right_hand_data = {'joint_positions':  KIN.motor2joint(0, +3, 0, np.array(self.dxl_filtered_position_list)),
        #                        'joint_velocities': KIN.motor2joint(0, +3, 1, np.array(self.dxl_filtered_velocity_list)),
        #                        #    'joint_torques':    KIN.motor2joint(0, +3, 2, np.array(self.dxl_filtered_current_list)),
        #                        'dxl_temperatures': np.array(self.dxl_temperature_list),
        #                        'dxl_voltages':     np.array(self.dxl_voltage_list)}
        #     MM.RIGHT_HAND_JOINT_STATE.set(right_hand_data)
        #     left_hand_data = {'joint_positions':  KIN.motor2joint(0, -3, 0, np.array(self.dxl_filtered_position_list)),
        #                       'joint_velocities': KIN.motor2joint(0, -3, 1, np.array(self.dxl_filtered_velocity_list)),
        #                       #   'joint_torques':    KIN.motor2joint(0, +3, 2, np.array(self.dxl_filtered_current_list)),
        #                       'dxl_temperatures': np.array(self.dxl_temperature_list),
        #                       'dxl_voltages':     np.array(self.dxl_voltage_list)}
        #     MM.LEFT_HAND_JOINT_STATE.set(left_hand_data)
        #     pos_data = np.concatenate((KIN.joint2motor(0, +3, 0, positions[0:7]), KIN.joint2motor(0, -3, 0, positions[7:14])))

        goal_position_list = [[int(pos_data[idx])] for idx in self.dxl_num_range]
        # print(goal_position_list)
        self.dam.sync_write(self.gsw_goal_position, goal_position_list)

    def set_enable_status(self, values):
        """
        Set DXL enable status
        """
        if self.dxl_enable_status_list != values:
            self.dam.sync_write(self.gsw_torque_enable, [[int(val)] for val in values])
            time.sleep(0.01)
            dxl_enable_info = self.dam.sync_read(self.gsr_torque_enable)
            if dxl_enable_info is not None:
                self.dxl_enable_status_list = [val[0] for val in dxl_enable_info]

            if self.dxl_enable_status_list == values:
                print("Setting Enable Status Succeed. Current Enable Status:", self.dxl_enable_status_list, end='\n\n')
            else:
                print("Setting Enable Status Failed. Current Enable Status:", self.dxl_enable_status_list, end='\n\n')

    def set_operating_mode(self, values):
        """
        Set DXL operating mode
        """
        if self.dxl_operating_mode_list != values:
            self.dam.sync_write(self.gsw_operating_mode, [[int(val)] for val in values])
            time.sleep(0.01)
            dxl_mode_info = self.dam.sync_read(self.gsr_operating_mode)
            if dxl_mode_info is not None:
                self.dxl_operating_mode_list = [val[0] for val in dxl_mode_info]

            if self.dxl_operating_mode_list == values:
                print("Setting Operating Mode Succeed. Current Operating Mode:", self.dxl_operating_mode_list, end='\n\n')
            else:
                print("Setting Operating Mode Failed. Current Operating Mode:", self.dxl_operating_mode_list, end='\n\n')


def main_loop():
    Themis = RDS.RobotDataManager()

    read_write_frequency                     = dcm.read_write_frequency
    update_temperature_and_voltage_frequency = 1
    check_thread_frequency                   = 10

    read_write_duration                     = 1. / read_write_frequency
    update_temperature_and_voltage_duration = 1. / update_temperature_and_voltage_frequency
    check_thread_duration                   = 1. / check_thread_frequency

    last_read_write_time                     = time.perf_counter()
    last_update_temperature_and_voltage_time = time.perf_counter()
    last_check_thread_time                   = time.perf_counter()

    thread_check = False

    t0 = time.perf_counter()
    while True:
        # time info
        loop_start_time = time.perf_counter()

        # check threading issue
        last_check_thread_elapse = loop_start_time - last_check_thread_time
        if last_check_thread_elapse > check_thread_duration:
            last_check_thread_time = loop_start_time
            if Themis.thread_issue(thread_name=chain_name):
                Themis.stop_threading()

        if not thread_check and loop_start_time - t0 > 1:
            cprint("THREAD IS DOING GREAT!", 'green')
            MM.THREAD_STATE.set({chain_name: np.array([1])}, opt='update')  # thread is running
            thread_check = True
        
        # update temperature & voltage
        last_update_temperature_and_voltage_elapse = loop_start_time - last_update_temperature_and_voltage_time
        if last_update_temperature_and_voltage_elapse > update_temperature_and_voltage_duration:
            last_update_temperature_and_voltage_time = loop_start_time
            dcm.update_temperature_and_voltage = True
        
        # read/write DXL
        last_read_write_elapse = loop_start_time - last_read_write_time
        if last_read_write_elapse > read_write_duration:
            last_read_write_time = loop_start_time

            if dcm.chain == CHAIN_HAND_R:
                commands = MM.RIGHT_HAND_JOINT_COMMAND.get()
            elif dcm.chain == CHAIN_HAND_L:
                commands = MM.LEFT_HAND_JOINT_COMMAND.get()
            # elif dcm.chain == CHAIN_HAND:
            #     right_hand_commands = MM.RIGHT_HAND_JOINT_COMMAND.get()
            #     left_hand_commands = MM.LEFT_HAND_JOINT_COMMAND.get()
            #     for key in list(commands.keys()):
            #         commands[key] = np.concatenate((right_hand_commands[key], left_hand_commands[key]))

            # need to change mode first before sending command
            dcm.set_operating_mode(values=commands['dxl_operating_modes'].tolist())

            dcm.read_write_dxl(positions=commands['goal_joint_positions'],
                               p_gains=commands['dxl_proportional_gains'], d_gains=commands['dxl_derivative_gains'])

            dcm.set_enable_status(values=commands['dxl_enable_statuses'].tolist())

            if last_read_write_elapse > read_write_duration * 1.5:
                cprint("Delayed " + str(1e3 * (last_read_write_elapse - read_write_duration))[0:5] + " ms", 'yellow')


if __name__ == '__main__':
    chain = input("Communicate right hand (rh) or left hand (lh) chain? ")
    if chain == 'rh':
        dcm = DXLControlManager(chain=CHAIN_HAND_R)
        chain_name = 'dxl_right_hand'
        MM.RIGHT_HAND_JOINT_COMMAND.set(opt='default')
        commands = MM.RIGHT_HAND_JOINT_COMMAND.get()
        print("====== THEMIS Right Hand DXL Actuator Communication Thread is running at", dcm.read_write_frequency, "Hz... ======")
    elif chain == 'lh':
        dcm = DXLControlManager(chain=CHAIN_HAND_L)
        chain_name = 'dxl_left_hand'
        MM.LEFT_HAND_JOINT_COMMAND.set(opt='default')
        commands = MM.LEFT_HAND_JOINT_COMMAND.get()
        print("====== THEMIS Left Hand DXL Actuator Communication Thread is running at",  dcm.read_write_frequency, "Hz... ======")

    # main_loop()
    try:
        main_loop()
    except RDS.DXL_ERROR:
        cprint("DXL IN ERROR! TERMINATE NOW!", 'red')
        MM.THREAD_STATE.set({chain_name: np.array([3])}, opt='update')  # DXL in error
    except RDS.THREAD_STOP:
        cprint("THREAD STOPPED PEACEFULLY!", 'light_grey')
        MM.THREAD_STATE.set({chain_name: np.array([0])}, opt='update')  # thread is stopped
    except (Exception, KeyboardInterrupt) as error:
        print(error)
        cprint("THREAD IN ERROR! TERMINATE NOW!", 'red')
        MM.THREAD_STATE.set({chain_name: np.array([2])}, opt='update')  # thread in error
    finally:
        try:
            dcm.set_enable_status([0] * dcm.dxl_num)
        except:
            pass