#!usr/bin/env python
__author__    = "Westwoodrobotics"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "April 9, 2025"
__update__    = "Aug 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
User Input
'''

import os
import time
import select
import subprocess
import tty, sys, termios
import Setting.robot_data as RDS
import Startup.memory_manager as MM
import Play.Navigation.memory_manager as MMN
import Library.MATH_FUNCTION.math_function as MF
from Play.config import *
from collections import defaultdict
from termcolor import colored, cprint
from Play.Locomotion.locomotion_macros import *
from Test.Actuator.set_joint import move_to_goal_joint_positions


class UserCommand(object):
    def __init__(self, robot=RDS.RobotDataManager(), controller='keyboard', command_frequency=20, display_frequency=10):
        # robot class
        self.robot = robot

        # setup
        self.controller = controller
        self.loop_freq  = command_frequency
        self.display_dt = 1 / display_frequency
        self.last_display_time = time.time()
        self.loop_count = 0

        # user command
        self.in_operate = True
        self.in_recover = False

        # navigation command
        self.navigation_mode = False
        self.apf_walk = False
        self.navigation_stand = False
        self.HEAD_PITCH = -0.3
        self.HEAD_YAW   = 0.0

        self.HEAD_PITCH_FILTERED = -0.6
        self.HEAD_YAW_FILTERED   = 0.0


        # operate parameter
        self.mode        = BALANCE
        self.target_mode = BALANCE
        self.mode_name   = {BALANCE: 'BALANCE',
                            WALK:    'WALK'}
        self.parameter = defaultdict(lambda: defaultdict(int))
        for idx in PARAMETER_ID_LIST:
            self.parameter[idx]['value_raw']    = PARAMETER_DEFAULT[idx]
            self.parameter[idx]['value']        = PARAMETER_DEFAULT[idx]
            self.parameter[idx]['value_filter'] = PARAMETER_DEFAULT[idx]

        self.posture_id = -1
        self.posture_start_time = 0

        self.path_following = False

        # misc
        self.tol         = 1e-3
        self.screen_flag = True if 'Themis' in subprocess.check_output(["screen -ls; true"], shell=True).decode("utf-8") else False

        # initialize the controller
        if self.controller == 'keyboard':
            self.filedes = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)
            print("Press any key to continue!")
            cmd = sys.stdin.read(1)[0]
            termios.tcflush(sys.stdin, termios.TCIOFLUSH)  # clear buffer
        elif self.controller == 'gamepad':
            Themis.update_gamepad_states()
        
    def terminate(self):
        if self.controller == 'keyboard':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.filedes)
        elif self.controller == 'gamepad':
            pass

    def update(self):
        if self.in_recover:
            self.recover()
        elif self.in_operate:
            self.operate()
            if self.path_following:
                self.self_driving()

        self.set_command()

        if time.time() - self.posture_start_time > 1.:
            self.posture_id = -1

        if time.time() - self.last_display_time > self.display_dt:
            self.last_display_time = time.time()
            self.display()

        self.loop_count = 0 if self.loop_count > 1e8 else self.loop_count + 1

    def recover(self):
        self.in_recover = False
        for idx in PARAMETER_ID_LIST:
            if self.mode in PARAMETER_MODE_LIST[idx]:
                if PARAMETER_RECOVER[idx] == 'y':
                    if np.abs(PARAMETER_DEFAULT[idx] - self.parameter[idx]['value_raw']) > self.tol:
                        self.in_recover = True
                        if self.loop_count % int(self.loop_freq / 20) == 0:  # 20 Hz max
                            self.parameter[idx]['value_raw'] = PARAMETER_DEFAULT[idx] if np.abs(PARAMETER_DEFAULT[idx] - self.parameter[idx]['value_raw']) < PARAMETER_INCREMENT[idx] else self.parameter[idx]['value_raw'] + np.sign(PARAMETER_DEFAULT[idx] - self.parameter[idx]['value_raw']) * PARAMETER_INCREMENT[idx]
                            self.parameter[idx]['value'] = self.parameter[idx]['value_raw']
        
        if self.in_recover is False:
            if self.mode != self.target_mode:
                # time.sleep(0.5)
                self.mode = self.target_mode
                # time.sleep(1)

    def operate(self):
        self.robot.update_gui_states()
        if self.robot.gui['walk']:
            self.target_mode = WALK
            self.path_following = False
        elif self.robot.gui['stand']:
            self.target_mode = BALANCE
            self.path_following = False

        if self.controller == 'keyboard':
            # read keyboard
            is_input, _, _ = select.select([sys.stdin], [], [], 0.01)  # adding timeout to this keyboard input
            cmd = sys.stdin.read(1)[0] if is_input else None

            if cmd == 'r':
                self.in_recover = True
                self.posture_id = 0
                self.posture_start_time = time.time()

            if cmd == ' ':
                is_input, _, _ = select.select([sys.stdin], [], [], 0.2)
                cmd = sys.stdin.read(1)[0] if is_input else None
                if cmd == '0':
                    self.target_mode = BALANCE
                    self.path_following = False
                elif cmd == '1':
                    self.target_mode = WALK
                    self.path_following = False
                # elif cmd == '2':
                #     self.target_mode = RUN
                #     self.path_following = False
                elif cmd == '6':
                    self.target_mode = WALK
                    self.path_following = True
            elif cmd == '1':
                self.posture_id = 1
                self.posture_start_time = time.time()
            elif cmd == '2':
                self.posture_id = 2
                self.posture_start_time = time.time()
            elif cmd == '3':
                self.posture_id = 3
                self.posture_start_time = time.time()
            elif cmd == '4':
                self.posture_id = 4
                self.posture_start_time = time.time()
            elif cmd == '5':
                self.posture_id = 5
                self.posture_start_time = time.time()
            elif cmd == '6':
                self.posture_id = 6
                self.posture_start_time = time.time()
            elif cmd == '7':
                self.posture_id = 7
                self.posture_start_time = time.time()
            elif cmd == '8':
                self.posture_id = 8
                self.posture_start_time = time.time()
            elif cmd == '9':
                self.posture_id = 9
                self.posture_start_time = time.time()
            elif cmd == '-':
                self.posture_id = 10
                self.posture_start_time = time.time()
            elif cmd == '=':
                self.posture_id = 11
                self.posture_start_time = time.time()
            elif cmd == '[':
                self.posture_id = 12
                self.posture_start_time = time.time()

            termios.tcflush(sys.stdin, termios.TCIOFLUSH)  # clear buffer
                
            for idx in PARAMETER_ID_LIST:
                if self.mode in PARAMETER_MODE_LIST[idx]:
                    if not self.path_following or (self.path_following and PARAMETER_PATH_FOLLOW[idx] == 'y'):
                        if cmd == PARAMETER_BUTTON_PLUS[idx] and PARAMETER_MAX[idx] - self.parameter[idx]['value_raw'] > self.tol:
                            self.parameter[idx]['value_raw'] += PARAMETER_INCREMENT[idx]
                        elif cmd == PARAMETER_BUTTON_MINUS[idx] and self.parameter[idx]['value_raw'] - PARAMETER_MIN[idx] > self.tol:
                            self.parameter[idx]['value_raw'] -= PARAMETER_INCREMENT[idx]
        elif self.controller == 'gamepad':

            Themis.update_gamepad_states()

            if Themis.gamepad['LZ'] and Themis.gamepad['RZ'] :
                Themis.stop_threading()

            if Themis.gamepad['ST']:
                if Themis.gamepad['BK']:
                    self.target_mode = BALANCE
                    self.path_following = False
                elif Themis.gamepad['Y']:
                    self.target_mode = WALK
                    self.path_following = False
                # elif Themis.gamepad['X']:
                #     self.target_mode = RUN
                #     self.path_following = False
                elif Themis.gamepad['A']:
                    self.target_mode = WALK
                    self.path_following = True
                
                #press START+U launch navigation mode
                elif Themis.gamepad['X']:
                    self.navigation_mode = True
                    MMN.NAVIGATION_COMMAND.set({'navigation_mode': np.array([1.0])})
                    

                if self.navigation_mode == True:
                    if Themis.gamepad['Y']:
                        self.apf_walk = True
                    elif Themis.gamepad['A']:
                        self.apf_walk = False
                    
                    if Themis.gamepad['ST'] and Themis.gamepad['BK']:
                        self.navigation_stand = True

                if not self.path_following:
                    if self.mode == WALK:
                        if Themis.gamepad['LSP'] and PARAMETER_MAX[FOOT_YAW_LEFT] - self.parameter[FOOT_YAW_LEFT]['value_raw'] > self.tol:
                            self.parameter[FOOT_YAW_LEFT]['value_raw'] += PARAMETER_INCREMENT[FOOT_YAW_LEFT]
                        elif Themis.gamepad['LSM'] and self.parameter[FOOT_YAW_LEFT]['value_raw'] - PARAMETER_MIN[FOOT_YAW_LEFT] > self.tol:
                            self.parameter[FOOT_YAW_LEFT]['value_raw'] -= PARAMETER_INCREMENT[FOOT_YAW_LEFT]

                        if Themis.gamepad['RSP'] and PARAMETER_MAX[FOOT_YAW_RIGHT] - self.parameter[FOOT_YAW_RIGHT]['value_raw'] > self.tol:
                            self.parameter[FOOT_YAW_RIGHT]['value_raw'] += PARAMETER_INCREMENT[FOOT_YAW_RIGHT]
                        elif Themis.gamepad['RSM'] and self.parameter[FOOT_YAW_RIGHT]['value_raw'] - PARAMETER_MIN[FOOT_YAW_RIGHT] > self.tol:
                            self.parameter[FOOT_YAW_RIGHT]['value_raw'] -= PARAMETER_INCREMENT[FOOT_YAW_RIGHT]

                        if Themis.gamepad['U'] and PARAMETER_MAX[FOOT_CLEARANCE_X] - self.parameter[FOOT_CLEARANCE_X]['value_raw'] > self.tol:
                            self.parameter[FOOT_CLEARANCE_X]['value_raw'] += PARAMETER_INCREMENT[FOOT_CLEARANCE_X]
                        elif Themis.gamepad['D'] and self.parameter[FOOT_CLEARANCE_X]['value_raw'] - PARAMETER_MIN[FOOT_CLEARANCE_X] > self.tol:
                            self.parameter[FOOT_CLEARANCE_X]['value_raw'] -= PARAMETER_INCREMENT[FOOT_CLEARANCE_X]

                        if Themis.gamepad['R'] and PARAMETER_MAX[FOOT_CLEARANCE_Y] - self.parameter[FOOT_CLEARANCE_Y]['value_raw'] > self.tol:
                            self.parameter[FOOT_CLEARANCE_Y]['value_raw'] += PARAMETER_INCREMENT[FOOT_CLEARANCE_Y]
                        elif Themis.gamepad['L'] and self.parameter[FOOT_CLEARANCE_Y]['value_raw'] - PARAMETER_MIN[FOOT_CLEARANCE_Y] > self.tol:
                            self.parameter[FOOT_CLEARANCE_Y]['value_raw'] -= PARAMETER_INCREMENT[FOOT_CLEARANCE_Y]
            else:
                if Themis.gamepad['BK']:
                    self.in_recover = True
                    self.posture_id = 0

                if Themis.gamepad['LSM']:
                    self.posture_id = 1
                    self.posture_start_time = time.time()
                elif Themis.gamepad['LSP']:
                    self.posture_id = 2
                    self.posture_start_time = time.time()
                elif Themis.gamepad['X']:
                    self.posture_id = 3
                    self.posture_start_time = time.time()
                elif Themis.gamepad['Y']:
                    self.posture_id = 4
                    self.posture_start_time = time.time()
                elif Themis.gamepad['U']:
                    self.posture_id = 5
                    self.posture_start_time = time.time()
                elif Themis.gamepad['D']:
                    self.posture_id = 6
                    self.posture_start_time = time.time()
                elif Themis.gamepad['L']:
                    self.posture_id = 7
                    self.posture_start_time = time.time()
                elif Themis.gamepad['R']:
                    self.posture_id = 8
                    self.posture_start_time = time.time()
                elif Themis.gamepad['A']:
                    self.posture_id = 9
                    self.posture_start_time = time.time()
                elif Themis.gamepad['B']:
                    self.posture_id = 10
                    self.posture_start_time = time.time()
                if Themis.gamepad['RS2']:
                    self.posture_id = 11
                    self.posture_start_time = time.time()
                elif Themis.gamepad['LS2']:
                    self.posture_id = 12
                    self.posture_start_time = time.time()
                
                if self.navigation_mode:
                    #get raw_data from right joystick
                    navigation_command = MMN.NAVIGATION_COMMAND.get()
                    head_yaw = navigation_command['head_yaw'][0]
                    head_pitch = navigation_command['head_pitch'][0]

                    self.HEAD_PITCH = self.axis2value(Themis.gamepad['RY'] + head_pitch, 0.0, -0.6)
                    self.HEAD_YAW   = self.axis2value(Themis.gamepad['RX'] + head_yaw, 1.0, -1.0)

                    if Themis.gamepad['LSZ'] > 0:
                        self.parameter[BASE_YAW_RATE]['value_raw'] =  self.axis2value(Themis.gamepad['LSZ'], PARAMETER_MAX[BASE_YAW_RATE], 0)
                    elif Themis.gamepad['RSZ'] > 0:
                        self.parameter[BASE_YAW_RATE]['value_raw'] = -self.axis2value(Themis.gamepad['RSZ'], PARAMETER_MAX[BASE_YAW_RATE], 0)
                    else:
                        self.parameter[BASE_YAW_RATE]['value_raw'] = 0

                if self.mode == BALANCE:
                    # if Themis.gamepad['U'] and PARAMETER_MAX[COM_POSITION_X] - self.parameter[COM_POSITION_X]['value_raw'] > self.tol:
                    #     self.parameter[COM_POSITION_X]['value_raw'] += PARAMETER_INCREMENT[COM_POSITION_X]
                    # elif Themis.gamepad['D'] and self.parameter[COM_POSITION_X]['value_raw'] - PARAMETER_MIN[COM_POSITION_X] > self.tol:
                    #     self.parameter[COM_POSITION_X]['value_raw'] -= PARAMETER_INCREMENT[COM_POSITION_X]
                    self.parameter[COM_POSITION_Y]['value_raw'] = self.axis2value(-Themis.gamepad['LX'], PARAMETER_MAX[COM_POSITION_Y], PARAMETER_MIN[COM_POSITION_Y])
                    self.parameter[COM_POSITION_Z]['value_raw'] = self.axis2value(Themis.gamepad['LY'], PARAMETER_MAX[COM_POSITION_Z], PARAMETER_MIN[COM_POSITION_Z])

                    self.parameter[BASE_ORIENTATION_X]['value_raw'] = self.axis2value(Themis.gamepad['RX'], PARAMETER_MAX[BASE_ORIENTATION_X], PARAMETER_MIN[BASE_ORIENTATION_X])
                    self.parameter[BASE_ORIENTATION_Y]['value_raw'] = self.axis2value(Themis.gamepad['RY'], PARAMETER_MAX[BASE_ORIENTATION_Y], PARAMETER_MIN[BASE_ORIENTATION_Y])
                    if Themis.gamepad['RSP'] and PARAMETER_MAX[BASE_ORIENTATION_Z] - self.parameter[BASE_ORIENTATION_Z]['value_raw'] > self.tol:
                        self.parameter[BASE_ORIENTATION_Z]['value_raw'] += PARAMETER_INCREMENT[BASE_ORIENTATION_Z]
                    elif Themis.gamepad['RSM'] and self.parameter[BASE_ORIENTATION_Z]['value_raw'] - PARAMETER_MIN[BASE_ORIENTATION_Z] > self.tol:
                        self.parameter[BASE_ORIENTATION_Z]['value_raw'] -= PARAMETER_INCREMENT[BASE_ORIENTATION_Z]
                else:
                    self.parameter[COM_VELOCITY_X]['value_raw'] = self.axis2value(Themis.gamepad['LY'], PARAMETER_MAX[COM_VELOCITY_X], PARAMETER_MIN[COM_VELOCITY_X])
                    
                    if not self.path_following:
                        self.parameter[COM_VELOCITY_Y]['value_raw'] = self.axis2value(-Themis.gamepad['LX'], PARAMETER_MAX[COM_VELOCITY_Y], PARAMETER_MIN[COM_VELOCITY_Y])

                        # if Themis.gamepad['RSP'] and PARAMETER_MAX[BASE_YAW_RATE] - self.parameter[BASE_YAW_RATE]['value_raw'] > self.tol:
                        #     self.parameter[BASE_YAW_RATE]['value_raw'] += PARAMETER_INCREMENT[BASE_YAW_RATE]
                        # elif Themis.gamepad['RSM'] and self.parameter[BASE_YAW_RATE]['value_raw'] - PARAMETER_MIN[BASE_YAW_RATE] > self.tol:
                        #     self.parameter[BASE_YAW_RATE]['value_raw'] -= PARAMETER_INCREMENT[BASE_YAW_RATE]
                        if Themis.gamepad['LSZ'] > 0:
                           self.parameter[BASE_YAW_RATE]['value_raw'] =  self.axis2value(Themis.gamepad['LSZ'], PARAMETER_MAX[BASE_YAW_RATE], 0)
                        elif Themis.gamepad['RSZ'] > 0:
                           self.parameter[BASE_YAW_RATE]['value_raw'] = -self.axis2value(Themis.gamepad['RSZ'], PARAMETER_MAX[BASE_YAW_RATE], 0)
                        else:
                           self.parameter[BASE_YAW_RATE]['value_raw'] = 0

        for idx in PARAMETER_ID_LIST:
            self.parameter[idx]['value'] = self.parameter[idx]['value_raw']

        if self.mode != BALANCE:
            PARAMETER_MAX[COM_VELOCITY_X] = COM_VELOCITY_LIMIT[self.mode][0]
            PARAMETER_MIN[COM_VELOCITY_X] = COM_VELOCITY_LIMIT[self.mode][1]
            PARAMETER_MAX[COM_VELOCITY_Y] = COM_VELOCITY_LIMIT[self.mode][2]
            PARAMETER_MIN[COM_VELOCITY_Y] = COM_VELOCITY_LIMIT[self.mode][3]

        if self.target_mode == BALANCE:
            if self.mode != BALANCE:
                self.in_recover = True
        else:
            if self.mode == BALANCE:
                self.in_recover = True
            else:
                if COM_VELOCITY_LIMIT[self.target_mode][1]-0.01 <= self.parameter[COM_VELOCITY_X]['value_raw'] <= COM_VELOCITY_LIMIT[self.target_mode][0]+0.01 and COM_VELOCITY_LIMIT[self.target_mode][3]-0.01 <= self.parameter[COM_VELOCITY_Y]['value_raw'] <= COM_VELOCITY_LIMIT[self.target_mode][2]+0.01:
                    self.mode = self.target_mode
                else:
                    self.target_mode = self.mode

    def self_driving(self):
        # two-wheeled robot: https://medium.com/@sarim.mehdi.550/mapping-path-following-for-a-two-wheeled-robot-b8bd55214405
        self.robot.update_sense_states()
        th = np.arctan2(self.robot.imu_rotation_matrix[1, 0], self.robot.imu_rotation_matrix[0, 0])

        # self.robot.update_plan_states()
        # th = self.robot.des_yaw

        # parameters
        b = 0.2         # two-wheeled robot length, must be greater than 0, unit in [m]
        yaw_lim = 90.0  # yaw rate limit,                                   unit in [deg/s]
        yaw_max = 20.0  # zero speed when yaw rate greater than yaw_max,    unit in [deg/s]
        yaw_min = 5.0   # max  speed when yaw rate less    than yaw_min,    unit in [deg/s]

        yaw_lim = 20.0 if HARDWARE else 90.0

        # target moving direction in world frame
        vec = np.array([0.1, 0.1])
        nv = np.sqrt(vec[0]**2 + vec[1]**2)
        th_des = np.arctan2(vec[1], vec[0])

        # target speed in robot frame
        v_b = self.parameter[COM_VELOCITY_X]['value_raw']
        if v_b < 0:
            th = th + PI

        if np.abs(v_b) >= 1e-2 and nv > 1e-3:
            diff = th_des - np.arctan2(np.sin(th), np.cos(th))
            # diff = diff + PI2 if diff < 0 else diff
            
            if PI_2 < diff <= PI:
                v = 0
                w = yaw_lim
            elif -PI_2 < diff < -PI:
                v = 0
                w = -yaw_lim
            else:
                # target velocity in world frame
                v_w = np.abs(v_b) * np.array([np.cos(th_des), np.sin(th_des)])

                # two-wheeled robot velocity in robot frame
                v_2 = np.array([[ np.cos(th),     np.sin(th)],
                                [-np.sin(th) / b, np.cos(th) / b]]) @ v_w
                v = v_2[0] if v_b > 0 else -v_2[0]
                w = v_2[1] * RAD2DEG
                
                # velocity adjustment
                if np.abs(w) > yaw_max:
                    v = 0.0
                elif np.abs(w) < yaw_min:
                    pass
                else:
                    v = (yaw_max - np.abs(w)) / (yaw_max - yaw_min) * v

                w = MF.sat(w, -yaw_lim, +yaw_lim) if np.abs(diff) > 0.1 else 0.0

            self.parameter[COM_VELOCITY_X]['value'] = v
            self.parameter[BASE_YAW_RATE]['value']   = w

    def set_command(self):
        
        self.HEAD_PITCH_FILTERED = MF.exp_filter(self.HEAD_PITCH_FILTERED, self.HEAD_PITCH, 0.85)
        self.HEAD_YAW_FILTERED   = MF.exp_filter(self.HEAD_YAW_FILTERED,   self.HEAD_YAW,   0.85)
        head_pose = [-self.HEAD_YAW_FILTERED, self.HEAD_PITCH_FILTERED]
        
        navigation_command = MMN.NAVIGATION_COMMAND.get()

        horizontal_velocity = navigation_command['horizontal_velocity']
        navigation_yaw_rate = navigation_command['yaw_rate']
        in_aviliable_area   = navigation_command['in_avaliable_region']
        navigation_status   = navigation_command['navigation_status']
        navigation          = navigation_command['navigation']

        for idx in PARAMETER_ID_LIST:
            self.parameter[idx]['value_filter'] = MF.exp_filter(self.parameter[idx]['value_filter'], self.parameter[idx]['value'], 0.85)
        
        #under navigation mode
        if self.navigation_mode:

            Themis.goal_joint_position[JOINT_HEAD_YAW] = head_pose[0]
            Themis.goal_joint_position[JOINT_HEAD_PITCH] = head_pose[1]
            Themis.set_joint_positions(chains=[CHAIN_HEAD])

            #ST + BK while under navigation mode
            if self.navigation_stand:
                self.navigation_mode = False
                self.target_mode = BALANCE
                MMN.NAVIGATION_COMMAND.set({'navigation_mode': np.array([0.0])})
                self.navigation_stand = False



            #under auto navigation mode and get goal pose
            if navigation[0] == 1:
                #excuting 
                if navigation_status[0] == 2 or navigation_status[0] == 0:
                    self.target_mode = WALK
                else:
                    self.target_mode = BALANCE 

                input_data = {'locomotion_mode':       np.array([self.target_mode]),
                            'horizontal_velocity':     horizontal_velocity,
                            'yaw_rate':                navigation_yaw_rate
                            }

            #under apf mode
            if navigation[0] == 2:

                self.target_mode = WALK if self.apf_walk else BALANCE

                if in_aviliable_area[0] == 0:
                    input_data = {'locomotion_mode':   np.array([self.target_mode]),
                                'horizontal_velocity': np.array([self.parameter[COM_VELOCITY_X]['value_filter'], self.parameter[COM_VELOCITY_Y]['value_filter']]),
                                'yaw_rate':            np.array([self.parameter[BASE_YAW_RATE]['value_filter']]) * DEG2RAD,
                                }
                if in_aviliable_area[0] == 1:
                    input_data = {'locomotion_mode':       np.array([self.target_mode]),
                                'horizontal_velocity':     horizontal_velocity,
                                'yaw_rate':            np.array([self.parameter[BASE_YAW_RATE]['value_filter']]) * DEG2RAD,
                                } 
            

            if navigation[0] == 0:
                input_data = {'locomotion_mode':       np.array([self.target_mode]),}
        #general mode
        else:
            input_data = {'locomotion_mode':       np.array([self.mode]),
                        'horizontal_velocity':     np.array([self.parameter[COM_VELOCITY_X]['value_filter'], self.parameter[COM_VELOCITY_Y]['value_filter']]),
                        'yaw_rate':                np.array([self.parameter[BASE_YAW_RATE]['value_filter']]) * DEG2RAD,
                        'com_position_change':     np.array([self.parameter[COM_POSITION_X]['value_filter'], self.parameter[COM_POSITION_Y]['value_filter'], self.parameter[COM_POSITION_Z]['value_filter']]),
                        'base_euler_angle_change': np.array([self.parameter[BASE_ORIENTATION_X]['value_filter'], self.parameter[BASE_ORIENTATION_Y]['value_filter'], self.parameter[BASE_ORIENTATION_Z]['value_filter']]) * DEG2RAD,
                        'right_foot_yaw_change':   np.array([self.parameter[FOOT_YAW_RIGHT]['value_filter']]) * DEG2RAD,
                        'left_foot_yaw_change':    np.array([self.parameter[FOOT_YAW_LEFT]['value_filter']])  * DEG2RAD,
                        'cop_clearance_change':    np.array([self.parameter[FOOT_CLEARANCE_X]['value_filter'], self.parameter[FOOT_CLEARANCE_Y]['value_filter']]),
                        'posture':                 np.array([self.posture_id])
                        }
        
        MM.USER_COMMAND.set(input_data)


    def display(self):
        # get BEAR info
        leg_r_data = MM.RIGHT_LEG_JOINT_STATE.get()
        leg_l_data = MM.LEFT_LEG_JOINT_STATE.get()
        vol_r_msg, vol_l_msg, tem_r_msg, tem_l_msg = " ", " ", " ", " "
        for idx in range(6):
            vol_r_i = leg_r_data['bear_voltages'][idx]
            vol_r_msg += (colored(str(vol_r_i)[0:4], 'red') if vol_r_i < 22 else str(vol_r_i)[0:4]) + " "

            vol_l_i = leg_l_data['bear_voltages'][idx]
            vol_l_msg += (colored(str(vol_l_i)[0:4], 'red') if vol_l_i < 22 else str(vol_l_i)[0:4]) + " "

            tem_r_i = leg_r_data['bear_temperatures'][idx]
            tem_r_msg += (colored(str(tem_r_i)[0:4], 'red') if vol_r_i > 75 else str(tem_r_i)[0:4]) + " "

            tem_l_i = leg_l_data['bear_temperatures'][idx]
            tem_l_msg += (colored(str(tem_l_i)[0:4], 'red') if tem_l_i > 75 else str(tem_l_i)[0:4]) + " "

        os.system('clear')
        print("====== The Navigation User Input Thread is running at", loop_freq, "Hz... ======")
        print()
        print("BEAR Voltage [V]:     ", vol_r_msg)
        print("                      ", vol_l_msg)
        print("BEAR Temperature [°C]:", tem_r_msg)
        print("                      ", tem_l_msg)
        print("_________")
        print()
        print("IN", self.mode_name[self.mode], "(press SPACEBAR + 0/1/2 to balance/walk/run)")
        print("_________")
        print()
        if self.navigation_mode:
            print("______________")
            print("Under Navigation Mode")
            print("______________")
        else:
            print("______________")
            print("Under User Input Mode")
            print("______________")
        if self.mode == BALANCE:
            print("Body Orientation")
            print("-      roll (T/Y):",  self.float2str(self.parameter[BASE_ORIENTATION_X]['value'], 'ang'), "[deg]")
            print("-     pitch (U/I):",  self.float2str(self.parameter[BASE_ORIENTATION_Y]['value'], 'ang'), "[deg]")
            print("-       yaw (O/P):",  self.float2str(self.parameter[BASE_ORIENTATION_Z]['value'], 'ang'), "[deg]")
            print()
            print("CoM Position")
            print("-  sagittal (F/G):",  self.float2str(self.parameter[COM_POSITION_X]['value'], 'len'), "[cm]")
            print("-   lateral (H/J):",  self.float2str(self.parameter[COM_POSITION_Y]['value'], 'len'), "[cm]")
            print("-  vertical (K/L):",  self.float2str(self.parameter[COM_POSITION_Z]['value'], 'len'), "[cm]")
        elif self.mode == WALK:
            print("CoM Velocity")
            print("-   sagittal (W/S):", self.float2str(self.parameter[COM_VELOCITY_X]['value'], 'len'), "[cm/s]")
            print("-    lateral (A/D):", self.float2str(self.parameter[COM_VELOCITY_Y]['value'], 'len'), "[cm/s]")
            print("-        yaw (Q/E):", self.float2str(self.parameter[BASE_YAW_RATE]['value'],   'ang'), "[deg/s]")
            print()
            print("Swing Foot")
            print("-  right yaw (Z/X):", self.float2str(self.parameter[FOOT_YAW_RIGHT]['value'],   'ang'), "[deg]")
            print("-   left yaw (C/V):", self.float2str(self.parameter[FOOT_YAW_LEFT]['value'],    'ang'), "[deg]")
            print("- foot clr x (N/M):", self.float2str(self.parameter[FOOT_CLEARANCE_X]['value'], 'len'), "[cm]")
            print("- foot clr y (</>):", self.float2str(self.parameter[FOOT_CLEARANCE_Y]['value'], 'len'), "[cm]")
        print()
        print("Press R to recover.")
        if self.screen_flag:
            cprint("Press Ctrl+A+D to go back to the terminal.", 'yellow')

    @staticmethod
    def axis2value(x, x_max, x_min):
        n = 1
        if x > 0:
            val = x**n * x_max
        else:
            val = (-x)**n * x_min
        return val
    
    @staticmethod
    def float2str(f, t='default'):
        if t == 'len':
            f = round(f * 1000) / 10
        else:
            f = round(f * 100) / 100

        if f >= 100:
            s = '+' + str(f)[:-1]
        elif f >= 10:
            s = '+' + str(f)
        elif 0 < f < 10:
            s = '+' + str(f) + '0'
        elif f == 0:
            s = ' ' + str(f) + '0'
        elif -10 < f < 0:
            s = str(f) + '0'
        else:
            s = str(f)

        return s


def main_loop():
    check_thread_frequency = 10
    check_thread_duration  = 1. / check_thread_frequency

    last_check_thread_time = time.time()

    thread_check = False

    # if uc.HEAD_PITCH == 0.0 and uc.HEAD_YAW == 0.0:
    #     head_pose = [0.0, -0.5]
    # else:
    #     head_pose = [uc.HEAD_PITCH_FILTERED, uc.HEAD_YAW_FILTERED]
    # print(head_pose)
    # move_to_goal_joint_positions(Themis, 200, 0.01, head_goal_positions=np.array(head_pose))
    
    t0 = time.time()
    try:
        while True:
            # time info
            loop_start_time = time.time()

            # check threading issue
            last_check_thread_elapse = loop_start_time - last_check_thread_time
            if last_check_thread_elapse > check_thread_duration:
                last_check_thread_time = loop_start_time
                if Themis.thread_issue():
                    Themis.stop_threading()

            if not thread_check and loop_start_time - t0 > 1:
                MM.THREAD_STATE.set({'top_level': np.array([1])}, opt='update')  # thread is running
                thread_check = True

            # update user command
            uc.update()





            # check time to ensure the thread stays at a consistent running loop
            while time.time() - loop_start_time < loop_duration:
                pass
    finally:
        uc.terminate()


if __name__ == '__main__':
    # Themis Setup
    Themis = RDS.RobotDataManager()

    # Control Frequency
    loop_freq     = 20  # run     at 20 Hz
    display_freq  = 10  # display at 10 Hz
    loop_duration = 1 / loop_freq

    # User Command Input Setup
    ctrl = 'gamepad' if GAMEPAD or DECK else 'keyboard'
    uc = UserCommand(robot=Themis, controller=ctrl, display_frequency=display_freq)

    try:
        main_loop()
    except RDS.THREAD_STOP:
        cprint("THREAD STOPPED PEACEFULLY!", 'light_grey')
        MM.THREAD_STATE.set({'top_level': np.array([0])}, opt='update')  # thread is stopped
    except (Exception, KeyboardInterrupt) as error:
        print(error)
        cprint("THREAD IN ERROR! TERMINATE NOW!", 'red')
        MM.THREAD_STATE.set({'top_level': np.array([2])}, opt='update')  # thread in error
