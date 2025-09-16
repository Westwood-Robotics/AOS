#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 5, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Script for communication with BEAR actuators
'''

import time
import Setting.robot_data as RDS
import Startup.memory_manager as MM
import Library.MATH_FUNCTION.math_function as MF
import Library.ROBOT_MODEL.THEMIS_kinematics as KIN
from termcolor import cprint
from Setting.Macros.bear_macros import *
from Setting.Macros.model_macros import *
from Setting.Macros.constant_macros import *
from Library.BEAR_ACTUATOR import Manager as bear_actuator_manager


class BearControlManager(object):
    def __init__(self, chain=None):
        # THEMIS BEAR chain
        self.chain = chain

        # BEAR serial port
        self.bear_baudrate = BEAR_BAUDRATE
        if self.chain == CHAIN_LEG_R:
            self.bear_port    = LEG_R_BEAR_PORT
            self.bear_id_list = LEG_R_BEAR_ID_LIST
            self.read_write_frequency = 1000
            self.bear_reenable_id_list = [BEAR_ANKLE_IN_R, BEAR_ANKLE_OUT_R]
        elif self.chain == CHAIN_LEG_L:
            self.bear_port    = LEG_L_BEAR_PORT
            self.bear_id_list = LEG_L_BEAR_ID_LIST
            self.read_write_frequency = 1000
            self.bear_reenable_id_list = [BEAR_ANKLE_IN_L, BEAR_ANKLE_OUT_L]
        elif self.chain == CHAIN_ARM_R:
            self.bear_port    = ARM_R_BEAR_PORT
            self.bear_id_list = ARM_R_BEAR_ID_LIST
            self.read_write_frequency = 500
            self.bear_reenable_id_list = list(ARM_R_BEAR_ID_LIST)
        elif self.chain == CHAIN_ARM_L:
            self.bear_port    = ARM_L_BEAR_PORT
            self.bear_id_list = ARM_L_BEAR_ID_LIST
            self.read_write_frequency = 500
            self.bear_reenable_id_list = list(ARM_L_BEAR_ID_LIST)
        elif self.chain == CHAIN_HEAD:
            self.bear_port    = HEAD_BEAR_PORT
            self.bear_id_list = HEAD_BEAR_ID_LIST
            self.read_write_frequency = 100
            self.bear_reenable_id_list = list(HEAD_BEAR_ID_LIST)

        # BEAR manager
        self.bam = bear_actuator_manager.BearActuatorManager(port=self.bear_port, baudrate=self.bear_baudrate)

        # BEAR info
        self.bear_num       = len(self.bear_id_list)
        self.bear_num_range = range(self.bear_num)

        self.bear_temperature_list = [20.] * self.bear_num
        self.bear_voltage_list     = [30.] * self.bear_num

        self.bear_operating_mode_list = [self.bam.operating_mode['torque']] * self.bear_num
        self.bear_enable_status_list  = [self.bam.enable_status['disable']] * self.bear_num

        self.bear_proportional_gain_list = [0.] * self.bear_num
        self.bear_derivative_gain_list   = [0.] * self.bear_num

        self.bear_filtered_velocity_list = [0.] * self.bear_num
        self.bear_filtered_position_list = [0.] * self.bear_num
        self.bear_filtered_current_list  = [0.] * self.bear_num

        self.enable_error_count     = 0
        self.enable_error_threshold = 50

        self.communication_error_count     = 0
        self.communication_error_threshold = 100

        self.check_enable_status            = False
        self.update_temperature_and_voltage = False

        # initialization
        self.check_bear()
        self.setup_config()
        self.set_enable_status(values=self.bear_enable_status_list)
        self.set_operating_mode(values=self.bear_operating_mode_list)

    def check_bear(self):
        """
        Ping all BEARs
        """
        # ping
        offline = [True] * self.bear_num
        error   = [True] * self.bear_num
        for idx, bear_id in enumerate(self.bear_id_list):
            trial = 0
            print("Pinging BEAR %02d ..." % bear_id)
            while offline[idx]:
                ping_info = self.bam.ping([bear_id])
                if ping_info[0][0] is not None:
                    offline[idx] = False
                    print("Pinging BEAR %02d Succeed." % bear_id)
                    if ping_info[0][1] == 128:
                        error[idx] = False
                    else:
                        cprint(("BEAR %02d ERROR: " + self.bam.decode_error(ping_info[0][1])) % bear_id, 'red')
                        if ping_info[0][1] >> 3 & 1:  # E-Stop
                            cprint("Please release E-Stop and try again.", 'yellow')
                            break
                        else:
                            print("Try Clearing Error...")
                            for _ in range(10):
                                self.bam.set_enable_status([bear_id], 'disable')
                                time.sleep(0.1)
                                ping_info = self.bam.ping([bear_id])
                                time.sleep(0.1)
                                if ping_info[0][1] == 128:
                                    error[idx] = False
                                    print("Clearing Error Succeed.")
                                    break
                            if ping_info[0][1] != 128:  # still ERROR!!!
                                cprint("Clearing Error Failed!", 'red')
                else:
                    trial += 1
                    if trial == 2:
                        cprint("Pinging BEAR %02d Failed. Retrying ..." % bear_id, 'yellow')
                    elif trial >= 6:
                        cprint("BEAR %02d Offline!!!" % bear_id, 'red')
                        break
                    time.sleep(0.5)
            print()
            time.sleep(0.1)

        if sum(offline) != 0 or sum(error) != 0:
            raise RDS.BEAR_ERROR

    def setup_config(self):
        """
        Setup configuration registers, e.g., PID & limit
        """
        # PID
        for target in ['position', 'velocity', 'force', 'iq', 'id']:
            if target == 'position':
                bear_pid_target = BEAR_POSITION_PID
            elif target == 'velocity':
                bear_pid_target = BEAR_VELOCITY_PID
            elif target == 'force':
                bear_pid_target = BEAR_FORCE_PID
            elif target == 'iq':
                bear_pid_target = BEAR_IQ_PID
            elif target == 'id':
                bear_pid_target = BEAR_ID_PID

            bear_list       = self.bear_id_list
            bear_pid_list   = [BEAR[bear_pid_target][bear_id] for bear_id in bear_list]
            bear_check_list = [False] * len(bear_list)
            while bear_list != []:
                self.bam.set_pid(bear_list, target, bear_pid_list)
                time.sleep(0.001)
                bear_pid_info = self.bam.get_pid(bear_list, target)
                for idx in range(len(bear_list)):
                    if None not in bear_pid_info[idx]:
                        if np.max(np.abs(np.array(bear_pid_info[idx]) - np.array(bear_pid_list[idx]))) < 1e-3:
                            bear_check_list[idx] = True
                bear_list       = [bear_list[idx]     for idx, val in enumerate(bear_check_list) if val is False]
                bear_pid_list   = [bear_pid_list[idx] for idx, val in enumerate(bear_check_list) if val is False]
                bear_check_list = [False] * len(bear_list)

        # limit
        for target in ['position', 'i', 'temperature']:
            if target == 'position':
                bear_limit_target = BEAR_POSITION_LIMIT
            elif target == 'i':
                bear_limit_target = BEAR_CURRENT_LIMIT
            elif target == 'temperature':
                bear_limit_target = BEAR_TEMPERATURE_LIMIT

            bear_list       = self.bear_id_list
            bear_limit_list = [BEAR[bear_limit_target][bear_id] for bear_id in bear_list]
            bear_check_list = [False] * len(bear_list)
            while bear_list != []:
                self.bam.set_limit(bear_list, target, bear_limit_list)
                time.sleep(0.001)
                bear_limit_info = self.bam.get_limit(bear_list, target)
                for idx in range(len(bear_list)):
                    if None not in bear_limit_info[idx]:
                        if np.max(np.abs(np.array(bear_limit_info[idx]) - np.array(bear_limit_list[idx]))) < 1e-3:
                            bear_check_list[idx] = True
                bear_list       = [bear_list[idx]       for idx, val in enumerate(bear_check_list) if val is False]
                bear_limit_list = [bear_limit_list[idx] for idx, val in enumerate(bear_check_list) if val is False]
                bear_check_list = [False] * len(bear_list)

        print("Setting PID & Limit Succeed.", end='\n\n')

    def read_write_bear(self, positions, velocities, torques, p_gains, d_gains):
        """
        Get joint states and set joint commands
        """
        if self.chain == CHAIN_LEG_R:
            pos_data = KIN.joint2motor(0, +1, 0, positions)
            vel_data = KIN.joint2motor(0, +1, 1, velocities)
            cur_data = KIN.joint2motor(0, +1, 2, torques)
        elif self.chain == CHAIN_LEG_L:
            pos_data = KIN.joint2motor(0, -1, 0, positions)
            vel_data = KIN.joint2motor(0, -1, 1, velocities)
            cur_data = KIN.joint2motor(0, -1, 2, torques)
        elif self.chain == CHAIN_ARM_R:
            pos_data = KIN.joint2motor(0, +2, 0, positions)
            vel_data = KIN.joint2motor(0, +2, 1, velocities)
            cur_data = KIN.joint2motor(0, +2, 2, torques)
        elif self.chain == CHAIN_ARM_L:
            pos_data = KIN.joint2motor(0, -2, 0, positions)
            vel_data = KIN.joint2motor(0, -2, 1, velocities)
            cur_data = KIN.joint2motor(0, -2, 2, torques)
        elif self.chain == CHAIN_HEAD:
            pos_data = KIN.joint2motor(0, 0, 0, positions)
            vel_data = KIN.joint2motor(0, 0, 1, velocities)
            cur_data = KIN.joint2motor(0, 0, 2, torques)
        write_name_list = ['goal_position', 'goal_velocity', 'goal_iq']
        write_data_list = [[pos_data[idx], vel_data[idx], cur_data[idx]] for idx in self.bear_num_range]

        read_name_list = ['present_position', 'present_velocity', 'present_iq']
        if self.check_enable_status:
            read_name_list += ['torque_enable']
        if self.update_temperature_and_voltage:
            read_name_list += ['winding_temperature', 'input_voltage']

        try:
            for idx, bear_id in enumerate(self.bear_id_list):
                self.bam.single_write(bear_id, ['p_gain_force', 'd_gain_force'], [p_gains[idx], d_gains[idx]])

            info = self.bam.bulk_read_write_stat(self.bear_id_list, read_name_list, write_name_list, write_data_list)
            for idx in self.bear_num_range:
                self.bear_filtered_position_list[idx] = MF.exp_filter(self.bear_filtered_position_list[idx], info[idx][0], 0.1)
                self.bear_filtered_velocity_list[idx] = MF.exp_filter(self.bear_filtered_velocity_list[idx], info[idx][1], 0.9)
                self.bear_filtered_current_list[idx]  = MF.exp_filter(self.bear_filtered_current_list[idx],  info[idx][2], 0.1)

            # check BEAR enable status
            if self.check_enable_status:
                enable_error = False
                estop = False
                for idx, bear_id in enumerate(self.bear_id_list):
                    if info[idx][3] is not None and info[idx][3] != self.bear_enable_status_list[idx]:
                        if bear_id in self.bear_reenable_id_list and info[idx][3] == 0 and self.bear_enable_status_list[idx] == 1:
                            cprint("BEAR %02d ENABLE STATUS ERROR!!! " % bear_id + "Detected Enable Status: %d " % info[idx][3] + "Enable again!", 'yellow')
                            self.bam.single_write(bear_id, ['torque_enable'], [1])
                        else:
                            cprint("BEAR %02d ENABLE STATUS ERROR!!! " % bear_id + "Detected Enable Status: %d" % info[idx][3], 'yellow')
                            enable_error = True
                            # print(bear_id, info[idx][3])
                            if info[idx][3] == 3:
                                estop = True
                        break
                self.enable_error_count = self.enable_error_count + 1 if enable_error else 0
                if self.enable_error_count >= self.enable_error_threshold:
                    if estop:
                        raise RDS.BEAR_ESTOP
                    else:
                        cprint("ERROR ENABLE STATUS!!!", 'red')
                        raise RDS.BEAR_ERROR
                # self.check_enable_status = False
            
            # update temperature and voltage
            if self.update_temperature_and_voltage:
                for idx, bear_id in enumerate(self.bear_id_list):
                    self.bear_temperature_list[idx] = info[idx][len(read_name_list)-2]
                    self.bear_voltage_list[idx]     = info[idx][len(read_name_list)-1]
                    if self.bear_temperature_list[idx] > 75:
                        cprint("BEAR %02d HIGH TERPERATURE!!! " % bear_id + "Detected Temperature: {:.2f}°C".format(self.bear_temperature_list[idx]), 'yellow')
                    if self.bear_voltage_list[idx] < 24:
                        cprint("BEAR %02d LOW VOLTAGE!!! " % bear_id + "Detected Voltage: {:.2f}V".format(self.bear_voltage_list[idx]), 'yellow')
                # self.update_temperature_and_voltage = False
            
            # update joint states
            if self.chain == CHAIN_LEG_R:
                right_leg_data = {'joint_positions':   KIN.motor2joint(0, +1, 0, np.array(self.bear_filtered_position_list)),
                                  'joint_velocities':  KIN.motor2joint(0, +1, 1, np.array(self.bear_filtered_velocity_list)),
                                  'joint_torques':     KIN.motor2joint(0, +1, 2, np.array(self.bear_filtered_current_list)),
                                  'bear_temperatures': np.array(self.bear_temperature_list),
                                  'bear_voltages':     np.array(self.bear_voltage_list)}
                MM.RIGHT_LEG_JOINT_STATE.set(right_leg_data)
            elif self.chain == CHAIN_LEG_L:
                left_leg_data = {'joint_positions':   KIN.motor2joint(0, -1, 0, np.array(self.bear_filtered_position_list)),
                                 'joint_velocities':  KIN.motor2joint(0, -1, 1, np.array(self.bear_filtered_velocity_list)),
                                 'joint_torques':     KIN.motor2joint(0, -1, 2, np.array(self.bear_filtered_current_list)),
                                 'bear_temperatures': np.array(self.bear_temperature_list),
                                 'bear_voltages':     np.array(self.bear_voltage_list)}
                MM.LEFT_LEG_JOINT_STATE.set(left_leg_data)
            elif self.chain == CHAIN_ARM_R:
                right_arm_data = {'joint_positions':   KIN.motor2joint(0, +2, 0, np.array(self.bear_filtered_position_list)),
                                  'joint_velocities':  KIN.motor2joint(0, +2, 1, np.array(self.bear_filtered_velocity_list)),
                                  'joint_torques':     KIN.motor2joint(0, +2, 2, np.array(self.bear_filtered_current_list)),
                                  'bear_temperatures': np.array(self.bear_temperature_list),
                                  'bear_voltages':     np.array(self.bear_voltage_list)}
                MM.RIGHT_ARM_JOINT_STATE.set(right_arm_data)
            elif self.chain == CHAIN_ARM_L:
                left_arm_data = {'joint_positions':   KIN.motor2joint(0, -2, 0, np.array(self.bear_filtered_position_list)),
                                 'joint_velocities':  KIN.motor2joint(0, -2, 1, np.array(self.bear_filtered_velocity_list)),
                                 'joint_torques':     KIN.motor2joint(0, -2, 2, np.array(self.bear_filtered_current_list)),
                                 'bear_temperatures': np.array(self.bear_temperature_list),
                                 'bear_voltages':     np.array(self.bear_voltage_list)}
                MM.LEFT_ARM_JOINT_STATE.set(left_arm_data)
            elif self.chain == CHAIN_HEAD:
                head_data = {'joint_positions':   KIN.motor2joint(0, 0, 0, np.array(self.bear_filtered_position_list)),
                             'joint_velocities':  KIN.motor2joint(0, 0, 1, np.array(self.bear_filtered_velocity_list)),
                             'joint_torques':     KIN.motor2joint(0, 0, 2, np.array(self.bear_filtered_current_list)),
                             'bear_temperatures': np.array(self.bear_temperature_list),
                             'bear_voltages':     np.array(self.bear_voltage_list)}
                MM.HEAD_JOINT_STATE.set(head_data)

            self.communication_error_count = 0
        except RDS.BEAR_ERROR:
            raise RDS.BEAR_ERROR
        except RDS.BEAR_ESTOP:
            raise RDS.BEAR_ESTOP
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as error:
            print(error)
            print(info)
            cprint("Communication Fail!!!", 'red')
            self.communication_error_count += 1
            if self.communication_error_count >= self.communication_error_threshold:
                self.set_damping_mode()
                raise RDS.BEAR_ERROR

        if self.check_enable_status:
            self.check_enable_status = False
        if self.update_temperature_and_voltage:
            self.update_temperature_and_voltage = False

    def set_enable_status(self, values):
        """
        Set BEAR enable status
        """
        if self.bear_enable_status_list != values:
            try:
                bear_list        = self.bear_id_list
                bear_enable_list = [[int(values[idx])] for idx in self.bear_num_range]
                bear_check_list  = [False] * len(bear_list)
                while bear_list != []:
                    self.bam.bulk_write_stat(bear_list, ['torque_enable'], bear_enable_list)
                    time.sleep(0.01)
                    bear_enable_info = self.bam.get_enable_status(bear_list)
                    if None in bear_enable_info:
                        self.communication_error_count += 1
                        if self.communication_error_count >= self.communication_error_threshold:
                            raise RDS.BEAR_ERROR
                    else:
                        self.communication_error_count = 0
                    for idx in range(len(bear_list)):
                        if bear_enable_info[idx] == bear_enable_list[idx][0]:
                            bear_check_list[idx] = True
                    bear_list        = [bear_list[idx]        for idx, val in enumerate(bear_check_list) if val is False]
                    bear_enable_list = [bear_enable_list[idx] for idx, val in enumerate(bear_check_list) if val is False]
                    bear_check_list  = [False] * len(bear_list)
                    
                self.bear_enable_status_list = values
                print("Setting Enable Status Succeed. Current Enable Status:", self.bear_enable_status_list, end='\n\n')
            except:
                print("Setting Enable Status Failed. Current Enable Status:", self.bear_enable_status_list, end='\n\n')

    def set_damping_mode(self):
        """
        Set BEAR to damping mode
        """
        self.set_enable_status([3] * self.bear_num)

    def set_operating_mode(self, values):
        """
        Set BEAR operating mode
        """
        if self.bear_operating_mode_list != values:
            try:
                bear_list       = self.bear_id_list
                bear_mode_list  = [[int(values[idx])] for idx in self.bear_num_range]
                bear_check_list = [False] * len(bear_list)
                while bear_list != []:
                    for idx, bear_id in enumerate(bear_list):
                        self.bam.single_write(bear_id, ['mode'], bear_mode_list[idx])
                    time.sleep(0.001)
                    bear_mode_info = self.bam.get_operating_mode(bear_list)
                    if None in bear_mode_info:
                        self.communication_error_count += 1
                        if self.communication_error_count >= self.communication_error_threshold:
                            raise RDS.BEAR_ERROR
                    else:
                        self.communication_error_count = 0
                    for idx in range(len(bear_list)):
                        if bear_mode_info[idx] == bear_mode_list[idx][0]:
                            bear_check_list[idx] = True
                    bear_list       = [bear_list[idx]      for idx, val in enumerate(bear_check_list) if val is False]
                    bear_mode_list  = [bear_mode_list[idx] for idx, val in enumerate(bear_check_list) if val is False]
                    bear_check_list = [False] * len(bear_list)
                    
                self.bear_operating_mode_list = values
                print("Setting Operating Mode Succeed. Current Operating Mode:", self.bear_operating_mode_list, end='\n\n')
            except:
                print("Setting Operating Mode Failed. Current Operating Mode:", self.bear_operating_mode_list, end='\n\n')


def main_loop():
    Themis = RDS.RobotDataManager()

    read_write_frequency                     = bcm.read_write_frequency
    check_enable_status_frequency            = 50
    update_temperature_and_voltage_frequency = 1
    check_thread_frequency                   = 100

    read_write_duration                     = 1. / read_write_frequency
    check_enable_status_duration            = 1. / check_enable_status_frequency
    update_temperature_and_voltage_duration = 1. / update_temperature_and_voltage_frequency
    check_thread_duration                   = 1. / check_thread_frequency

    last_read_write_time                     = time.perf_counter()
    last_check_enable_status_time            = time.perf_counter()
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
        
        # check enable status
        last_check_enable_status_elapse = loop_start_time - last_check_enable_status_time
        if last_check_enable_status_elapse > check_enable_status_duration:
            last_check_enable_status_time = loop_start_time
            bcm.check_enable_status = True
        
        # update temperature & voltage
        last_update_temperature_and_voltage_elapse = loop_start_time - last_update_temperature_and_voltage_time
        if last_update_temperature_and_voltage_elapse > update_temperature_and_voltage_duration:
            last_update_temperature_and_voltage_time = loop_start_time
            bcm.update_temperature_and_voltage = True
        
        # read/write BEAR
        last_read_write_elapse = loop_start_time - last_read_write_time
        if last_read_write_elapse > read_write_duration:
            last_read_write_time = loop_start_time
            
            if bcm.chain == CHAIN_LEG_R:
                commands = MM.RIGHT_LEG_JOINT_COMMAND.get()
            elif bcm.chain == CHAIN_LEG_L:
                commands = MM.LEFT_LEG_JOINT_COMMAND.get()
            elif bcm.chain == CHAIN_ARM_R:
                commands = MM.RIGHT_ARM_JOINT_COMMAND.get()
            elif bcm.chain == CHAIN_ARM_L:
                commands = MM.LEFT_ARM_JOINT_COMMAND.get()
            elif bcm.chain == CHAIN_HEAD:
                commands = MM.HEAD_JOINT_COMMAND.get()

            # need to change mode first before sending command
            bcm.set_operating_mode(values=commands['bear_operating_modes'].tolist())

            bcm.read_write_bear(positions=commands['goal_joint_positions'], velocities=commands['goal_joint_velocities'], torques=commands['goal_joint_torques'],
                                p_gains=commands['bear_proportional_gains'].tolist(), d_gains=commands['bear_derivative_gains'].tolist())

            bcm.set_enable_status(values=commands['bear_enable_statuses'].tolist())

            if last_read_write_elapse > read_write_duration * 1.5:
                cprint("Delayed " + str(1e3 * (last_read_write_elapse - read_write_duration))[0:5] + " ms", 'yellow')


if __name__ == '__main__':
    try:
        chain = input("Communicate right leg (rl) or left leg (ll) or right arm (ra) or left arm (la) or head (h) chain? ")
        if chain == 'rl':
            chain_name = 'bear_right_leg'
            MM.RIGHT_LEG_JOINT_COMMAND.set(opt='default')
            commands = MM.RIGHT_LEG_JOINT_COMMAND.get()
            bcm = BearControlManager(chain=CHAIN_LEG_R)
            print("====== THEMIS Right Leg BEAR Actuator Communication Thread is running at",  bcm.read_write_frequency, "Hz... ======")
        elif chain == 'll':
            chain_name = 'bear_left_leg'
            MM.LEFT_LEG_JOINT_COMMAND.set(opt='default')
            commands = MM.LEFT_LEG_JOINT_COMMAND.get()
            bcm = BearControlManager(chain=CHAIN_LEG_L)

            print("====== THEMIS Left Leg BEAR Actuator Communication Thread is running at",   bcm.read_write_frequency, "Hz... ======")
        elif chain == 'ra':
            
            chain_name = 'bear_right_arm'
            MM.RIGHT_ARM_JOINT_COMMAND.set(opt='default')
            commands = MM.RIGHT_ARM_JOINT_COMMAND.get()
            bcm = BearControlManager(chain=CHAIN_ARM_R)
            print("====== THEMIS Right Arm BEAR Actuator Communication Thread is running at",  bcm.read_write_frequency, "Hz... ======")
        elif chain == 'la':
            
            chain_name = 'bear_left_arm'
            MM.LEFT_ARM_JOINT_COMMAND.set(opt='default')
            commands = MM.LEFT_ARM_JOINT_COMMAND.get()
            bcm = BearControlManager(chain=CHAIN_ARM_L)
            print("====== THEMIS Left Arm BEAR Actuator Communication Thread is running at",   bcm.read_write_frequency, "Hz... ======")
        elif chain == 'h':
            
            chain_name = 'bear_head'
            MM.HEAD_JOINT_COMMAND.set(opt='default')
            commands = MM.HEAD_JOINT_COMMAND.get()
            bcm = BearControlManager(chain=CHAIN_HEAD)
            print("====== THEMIS Head BEAR Actuator Communication Thread is running at",       bcm.read_write_frequency, "Hz... ======")

        main_loop()
    except RDS.BEAR_ERROR:
        cprint("BEAR IN ERROR! TERMINATE NOW!", 'red')
        MM.THREAD_STATE.set({chain_name: np.array([3])}, opt='update')  # BEAR in error
    except RDS.BEAR_ESTOP:
        cprint("BEAR IN E-STOP! TERMINATE NOW!", 'red')
        MM.THREAD_STATE.set({chain_name: np.array([4])}, opt='update')  # BEAR ESTOP
    except RDS.THREAD_STOP:
        cprint("THREAD STOPPED PEACEFULLY!", 'light_grey')
        MM.THREAD_STATE.set({chain_name: np.array([0])}, opt='update')  # thread is stopped
    except (Exception, KeyboardInterrupt) as error:
        print(error)
        cprint("THREAD IN ERROR! TERMINATE NOW!", 'red')
        MM.THREAD_STATE.set({chain_name: np.array([2])}, opt='update')  # thread in error
    finally:
        try:
            bcm.set_damping_mode()
        except:
            pass
