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
Script that holds useful robot info
'''

import time
import Startup.memory_manager as MM
import Library.MATH_FUNCTION.math_function as MF
import Library.ROBOT_MODEL.THEMIS_kinematics as KIN
from Play.config import *
from termcolor import cprint
from collections import defaultdict
from Setting.Macros.dxl_macros import *
from Setting.Macros.bear_macros import *
from Setting.Macros.model_macros import *
from Setting.Macros.locomotion_macros import *


class BEAR_ESTOP(Exception):
    """
    Raised when E-STOP signal is detected
    """
    pass


class BEAR_ERROR(Exception):
    """
    Raised when BEAR in error
    """
    pass


class DXL_ERROR(Exception):
    """
    Raised when DXL in error
    """
    pass


class THREAD_STOP(Exception):
    """
    Raised to stop thread
    """
    pass


class IMU_OFFLINE(Exception):
    """
    Raised when IMU connection timeout
    """
    pass


class ROBOT_FALLING(Exception):
    """
    Raised when robot is falling
    """
    pass


class ROBOT_NOT_ON_GROUND(Exception):
    """
    Raised when robot is not placed on the ground during initialization
    """
    pass


class RobotDataManager:
    def __init__(self, simulation=False):
        # Simulation or Hardware
        self.simulation = simulation

        # Time info
        self.start_time = self.get_system_time()

        # BEAR info
        self.bear_enable_status  = defaultdict(int)
        self.bear_operating_mode = defaultdict(int)

        self.bear_temperature = defaultdict(int)
        self.bear_voltage     = defaultdict(int)

        self.bear_proportional_gain = defaultdict(int)
        self.bear_derivative_gain   = defaultdict(int)
        for bear in ROBOT_BEAR_ID_LIST:
            self.bear_proportional_gain[bear] = BEAR[BEAR_FORCE_PID][bear][0]
            self.bear_derivative_gain[bear]   = BEAR[BEAR_FORCE_PID][bear][2]


        self.bear_enable_status = {'disable': 0, 'enable': 1, 'error': 2, 'damping': 3}
        self.bear_operating_mode = {'torque': 0, 'velocity': 1, 'position': 2, 'force': 3}

        # DXL info
        self.dxl_enable_status  = defaultdict(int)
        self.dxl_operating_mode = defaultdict(int)

        self.dxl_temperature = defaultdict(int)
        self.dxl_voltage     = defaultdict(int)

        self.dxl_proportional_gain = defaultdict(int)
        self.dxl_derivative_gain   = defaultdict(int)

        self.dxl_enable_status = {'disable': 0, 'enable': 1}
        self.dxl_operating_mode = {'torque': 0, 'velocity': 1, 'position': 3}

        # Joint info
        self.joint_position = defaultdict(int)
        self.joint_velocity = defaultdict(int)
        self.joint_torque   = defaultdict(int)

        self.goal_joint_position = defaultdict(int)
        self.goal_joint_velocity = defaultdict(int)
        self.goal_joint_torque   = defaultdict(int)

        # Battery info
        self.battery = defaultdict(int)

        # GUI info
        self.gui = defaultdict(int)

        # Gamepad info
        self.gamepad = defaultdict(int)
        
        # Robot Info
        # foot state
        self.R_wf     = defaultdict(int)
        self.w_ff     = defaultdict(int)
        self.Jw_ff    = defaultdict(int)
        self.dJwdq_ff = defaultdict(int)
        self.p_wf     = defaultdict(int)
        self.v_wf     = defaultdict(int)
        self.Jv_wf    = defaultdict(int)
        self.dJvdq_wf = defaultdict(int)
        # self.pico_foot_contact_state = defaultdict(int)
        self.est_foot_contact_state  = defaultdict(int)
        self.est_foot_contact_force  = defaultdict(int)
        
        # ankle state
        self.p_wa     = defaultdict(int)
        self.v_wa     = defaultdict(int)
        self.Jv_wa    = defaultdict(int)
        self.dJvdq_wa = defaultdict(int)

        # hand state
        self.R_wh     = defaultdict(int)
        self.w_hh     = defaultdict(int)
        self.Jw_hh    = defaultdict(int)
        self.dJwdq_hh = defaultdict(int)
        self.p_wh     = defaultdict(int)
        self.v_wh     = defaultdict(int)
        self.Jv_wh    = defaultdict(int)
        self.dJvdq_wh = defaultdict(int)

        # head state
        self.R_wt     = np.eye(3)
        self.w_tt     = np.zeros(3)
        self.Jw_tt    = np.zeros((3, 8))
        self.dJwdq_tt = np.zeros(3)
        self.p_wt     = np.zeros(3)
        self.v_wt     = np.zeros(3)
        self.Jv_wt    = np.zeros((3, 8))
        self.dJvdq_wt = np.zeros(3)

        # head camera state
        self.R_wc = np.eye(3)
        self.w_cc = np.zeros(3)
        self.p_wc = np.zeros(3)
        self.v_wc = np.zeros(3)

        # swing
        self.foot_phase = defaultdict(int)
        self.des_p_wa   = defaultdict(int)
        self.des_v_wa   = defaultdict(int)
        self.des_a_wa   = defaultdict(int)
        self.des_R_wf   = defaultdict(int)
        self.des_w_ff   = defaultdict(int)

        self.last_p_wa = defaultdict(int)
        self.goal_p_wa = defaultdict(int)

        self.cmd_foot_yaw_change = defaultdict(int)

        self.hand_phase = defaultdict(int)
        self.des_p_wh = defaultdict(int)
        self.des_v_wh = defaultdict(int)
        self.des_R_wh = defaultdict(int)
        self.des_w_hh = defaultdict(int)

    def update_battery_states(self):
        """
        Update battery states from shared memory
        """
        battery_data = MM.BATTERY_STATE.get()

        for key in battery_data:
            self.battery[key] = battery_data[key]

    def update_sense_states(self):
        """
        Update sense states from shared memory
        """
        sense_data = MM.SENSE_STATE.get()

        self.imu_acceleration    = sense_data['imu_acceleration']
        self.imu_angular_rate    = sense_data['imu_angular_rate']
        self.imu_rotation_matrix = sense_data['imu_rotation_matrix']

        # self.pico_foot_contact_state[RIGHT] = sense_data['foot_contact_state'][0, 0:3]
        # self.pico_foot_contact_state[LEFT]  = sense_data['foot_contact_state'][1, 0:3]
    
    def update_thread_states(self):
        """
        Update thread states from shared memory
        """
        self.thread_state = MM.THREAD_STATE.get()

    def update_thread_commands(self):
        """
        Update thread commands from shared memory
        """
        self.thread_command = MM.THREAD_COMMAND.get()

    def update_gamepad_states(self):
        """
        Update gamepad states from shared memory
        """
        gamepad_data = MM.GAMEPAD_STATE.get()

        for key in gamepad_data:
            self.gamepad[key] = gamepad_data[key][0]

    def update_locomotion_planner_states(self):
        """
        Update locomotion planner states from shared memory
        """
        locomotion_plan_data = MM.LOCOMOTION_PLANNER_STATE.get()

        self.locomotion_mode  = int(locomotion_plan_data['locomotion_mode'][0])
        self.locomotion_phase = int(locomotion_plan_data['locomotion_phase'][0])

        self.des_R_wb  = locomotion_plan_data['base_rotation_matrix']
        self.des_w_bb  = locomotion_plan_data['base_angular_rate']
        self.des_yaw   = locomotion_plan_data['base_heading'][0]
        self.des_R_yaw = MF.Rz(self.des_yaw)
        self.des_p_wb  = locomotion_plan_data['base_position']
        self.des_v_wb  = locomotion_plan_data['base_velocity']
        self.des_p_wg  = locomotion_plan_data['com_position']
        self.des_v_wg  = locomotion_plan_data['com_velocity']

        self.foot_phase[RIGHT] = locomotion_plan_data['right_foot_phase'][0]
        self.des_p_wa[RIGHT]   = locomotion_plan_data['right_ankle_position']
        self.des_v_wa[RIGHT]   = locomotion_plan_data['right_ankle_velocity']
        self.des_a_wa[RIGHT]   = locomotion_plan_data['right_ankle_acceleration']
        self.des_R_wf[RIGHT]   = locomotion_plan_data['right_foot_rotation_matrix']
        self.des_w_ff[RIGHT]   = locomotion_plan_data['right_foot_angular_rate']
        self.last_p_wa[RIGHT]  = locomotion_plan_data['right_ankle_last_position']
        self.goal_p_wa[RIGHT]  = locomotion_plan_data['right_ankle_goal_position']

        self.foot_phase[LEFT]  = locomotion_plan_data['left_foot_phase'][0]
        self.des_p_wa[LEFT]    = locomotion_plan_data['left_ankle_position']
        self.des_v_wa[LEFT]    = locomotion_plan_data['left_ankle_velocity']
        self.des_a_wa[LEFT]    = locomotion_plan_data['left_ankle_acceleration']
        self.des_R_wf[LEFT]    = locomotion_plan_data['left_foot_rotation_matrix']
        self.des_w_ff[LEFT]    = locomotion_plan_data['left_foot_angular_rate']
        self.last_p_wa[LEFT]   = locomotion_plan_data['left_ankle_last_position']
        self.goal_p_wa[LEFT]   = locomotion_plan_data['left_ankle_goal_position']

    def update_input_states(self):
        """
        Update user input states from shared memory
        """
        input_data = MM.USER_COMMAND.get()

        self.cmd_mode                   = int(input_data['locomotion_mode'][0])
        self.cmd_velocity               = np.copy(input_data['horizontal_velocity'])
        self.cmd_yaw_rate               = input_data['yaw_rate'][0]
        self.cmd_p_wg_change            = input_data['com_position_change']
        self.cmd_euler_ang_change       = input_data['base_euler_angle_change']
        self.cmd_R_wb_change            = MF.Rz(self.cmd_euler_ang_change[2]) @ MF.Ry(self.cmd_euler_ang_change[1]) @ MF.Rx(self.cmd_euler_ang_change[0])
        self.cmd_foot_yaw_change[RIGHT] = input_data['right_foot_yaw_change'][0]
        self.cmd_foot_yaw_change[LEFT]  = input_data['left_foot_yaw_change'][0]
        self.cmd_cop_clearance_change   = input_data['cop_clearance_change']

    def update_base_states(self):
        """
        Update base states from shared memory
        """
        base_data = MM.BASE_STATE.get()
        
        self.p_wb  = base_data['base_position']
        self.v_wb  = base_data['base_velocity']
        self.a_wb  = base_data['base_acceleration']
        self.R_wb  = base_data['base_rotation_matrix']
        self.w_bb  = base_data['base_angular_rate']
        self.yaw   = base_data['base_heading'][0]
        self.R_yaw = MF.Rz(self.yaw)

    def update_com_states(self):
        """
        Update CoM states from shared memory
        """
        com_data = MM.COM_STATE.get()

        self.p_wg  = com_data['com_position']
        self.v_wg  = com_data['com_velocity']
        self.l_wg  = com_data['linear_momentum']
        self.k_wg  = com_data['angular_momentum']
        self.k_wc  = com_data['angular_momentum_cop']
        self.H     = com_data['H_matrix']
        self.CG    = com_data['CG_vector']
        self.AG    = com_data['AG_matrix']
        self.dAGdq = com_data['dAGdq_vector']

    def update_right_foot_states(self):
        """
        Update right foot states from shared memory
        """
        right_foot_data = MM.RIGHT_FOOT_STATE.get()

        self.R_wf[RIGHT]     = right_foot_data['right_foot_rotation_matrix']
        self.w_ff[RIGHT]     = right_foot_data['right_foot_angular_rate']
        self.Jw_ff[RIGHT]    = right_foot_data['right_foot_Jw']
        self.dJwdq_ff[RIGHT] = right_foot_data['right_foot_dJwdq']
        self.p_wf[RIGHT]     = right_foot_data['right_foot_position']
        self.v_wf[RIGHT]     = right_foot_data['right_foot_velocity']
        self.Jv_wf[RIGHT]    = right_foot_data['right_foot_Jv']
        self.dJvdq_wf[RIGHT] = right_foot_data['right_foot_dJvdq']

        self.est_foot_contact_state[RIGHT] = right_foot_data['right_foot_contact_state'][0]
        self.est_foot_contact_force[RIGHT] = right_foot_data['right_foot_contact_force']

    def update_left_foot_states(self):
        """
        Update left foot states from shared memory
        """
        left_foot_data = MM.LEFT_FOOT_STATE.get()

        self.R_wf[LEFT]     = left_foot_data['left_foot_rotation_matrix']
        self.w_ff[LEFT]     = left_foot_data['left_foot_angular_rate']
        self.Jw_ff[LEFT]    = left_foot_data['left_foot_Jw']
        self.dJwdq_ff[LEFT] = left_foot_data['left_foot_dJwdq']
        self.p_wf[LEFT]     = left_foot_data['left_foot_position']
        self.v_wf[LEFT]     = left_foot_data['left_foot_velocity']
        self.Jv_wf[LEFT]    = left_foot_data['left_foot_Jv']
        self.dJvdq_wf[LEFT] = left_foot_data['left_foot_dJvdq']
        
        self.est_foot_contact_state[LEFT] = left_foot_data['left_foot_contact_state'][0]
        self.est_foot_contact_force[LEFT] = left_foot_data['left_foot_contact_force']

    def update_right_ankle_states(self):
        """
        Update right ankle states from shared memory
        """
        right_ankle_data = MM.RIGHT_ANKLE_STATE.get()

        self.p_wa[RIGHT]     = right_ankle_data['right_ankle_position']
        self.v_wa[RIGHT]     = right_ankle_data['right_ankle_velocity']
        self.Jv_wa[RIGHT]    = right_ankle_data['right_ankle_Jv']
        self.dJvdq_wa[RIGHT] = right_ankle_data['right_ankle_dJvdq']

    def update_left_ankle_states(self):
        """
        Update left ankle states from shared memory
        """
        left_ankle_data = MM.LEFT_ANKLE_STATE.get()

        self.p_wa[LEFT]     = left_ankle_data['left_ankle_position']
        self.v_wa[LEFT]     = left_ankle_data['left_ankle_velocity']
        self.Jv_wa[LEFT]    = left_ankle_data['left_ankle_Jv']
        self.dJvdq_wa[LEFT] = left_ankle_data['left_ankle_dJvdq']

    def update_right_hand_states(self):
        """
        Update right hand states from shared memory
        """
        right_hand_data = MM.RIGHT_HAND_STATE.get()

        self.R_wh[RIGHT]     = right_hand_data['right_hand_rotation_matrix']
        self.w_hh[RIGHT]     = right_hand_data['right_hand_angular_rate']
        self.Jw_hh[RIGHT]    = right_hand_data['right_hand_Jw']
        self.dJwdq_hh[RIGHT] = right_hand_data['right_hand_dJwdq']
        self.p_wh[RIGHT]     = right_hand_data['right_hand_position']
        self.v_wh[RIGHT]     = right_hand_data['right_hand_velocity']
        self.Jv_wh[RIGHT]    = right_hand_data['right_hand_Jv']
        self.dJvdq_wh[RIGHT] = right_hand_data['right_hand_dJvdq']

    def update_left_hand_states(self):
        """
        Update left hand states from shared memory
        """
        left_hand_data = MM.LEFT_HAND_STATE.get()

        self.R_wh[LEFT]     = left_hand_data['left_hand_rotation_matrix']
        self.w_hh[LEFT]     = left_hand_data['left_hand_angular_rate']
        self.Jw_hh[LEFT]    = left_hand_data['left_hand_Jw']
        self.dJwdq_hh[LEFT] = left_hand_data['left_hand_dJwdq']
        self.p_wh[LEFT]     = left_hand_data['left_hand_position']
        self.v_wh[LEFT]     = left_hand_data['left_hand_velocity']
        self.Jv_wh[LEFT]    = left_hand_data['left_hand_Jv']
        self.dJvdq_wh[LEFT] = left_hand_data['left_hand_dJvdq']

    def update_head_states(self):
        """
        Update head states from shared memory
        """
        head_data = MM.HEAD_STATE.get()

        self.R_wt     = head_data['head_rotation_matrix']
        self.w_tt     = head_data['head_angular_rate']
        self.Jw_tt    = head_data['head_Jw']
        self.dJwdq_tt = head_data['head_dJwdq']
        self.p_wt     = head_data['head_position']
        self.v_wt     = head_data['head_velocity']
        self.Jv_wt    = head_data['head_Jv']
        self.dJvdq_wt = head_data['head_dJvdq']

    def update_head_camera_states(self):
        """
        Update head camera states from shared memory
        """
        head_camera_data = MM.HEAD_CAMERA_STATE.get()

        self.R_wc = head_camera_data['head_camera_rotation_matrix']
        self.w_cc = head_camera_data['head_camera_angular_rate']
        self.p_wc = head_camera_data['head_camera_position']
        self.v_wc = head_camera_data['head_camera_velocity']

    # def update_robot_states(self):
    #     """
    #     Update robot states from shared memory
    #     """
    #     self.update_base_states()
    #     self.update_com_states()

    #     self.update_right_foot_states()
    #     self.update_left_foot_states()

    #     self.update_right_ankle_states()
    #     self.update_left_ankle_states()

    #     self.update_right_hand_states()
    #     self.update_left_hand_states()

    #     self.update_head_states()

    def update_joint_states(self, chains):
        """
        Update joint states from shared memory
        """
        if CHAIN_LEG_R in chains:
            right_leg_data = MM.RIGHT_LEG_JOINT_STATE.get()
            for idx, joint_id in enumerate(LEG_R_JOINT_ID_LIST):
                self.joint_position[joint_id] = right_leg_data['joint_positions'][idx]
                self.joint_velocity[joint_id] = right_leg_data['joint_velocities'][idx]
                self.joint_torque[joint_id]   = right_leg_data['joint_torques'][idx]
            for idx, bear_id in enumerate(LEG_R_BEAR_ID_LIST):
                self.bear_temperature[bear_id] = right_leg_data['bear_temperatures'][idx]
                self.bear_voltage[bear_id]     = right_leg_data['bear_voltages'][idx]

        if CHAIN_LEG_L in chains:
            left_leg_data = MM.LEFT_LEG_JOINT_STATE.get()
            for idx, joint_id in enumerate(LEG_L_JOINT_ID_LIST):
                self.joint_position[joint_id] = left_leg_data['joint_positions'][idx]
                self.joint_velocity[joint_id] = left_leg_data['joint_velocities'][idx]
                self.joint_torque[joint_id]   = left_leg_data['joint_torques'][idx]
            for idx, bear_id in enumerate(LEG_L_BEAR_ID_LIST):
                self.bear_temperature[bear_id] = left_leg_data['bear_temperatures'][idx]
                self.bear_voltage[bear_id]     = left_leg_data['bear_voltages'][idx]
        
        if CHAIN_ARM_R in chains:
            right_arm_data = MM.RIGHT_ARM_JOINT_STATE.get()
            for idx, joint_id in enumerate(ARM_R_JOINT_ID_LIST):
                self.joint_position[joint_id] = right_arm_data['joint_positions'][idx]
                self.joint_velocity[joint_id] = right_arm_data['joint_velocities'][idx]
                self.joint_torque[joint_id]   = right_arm_data['joint_torques'][idx]
            for idx, bear_id in enumerate(ARM_R_BEAR_ID_LIST):
                self.bear_temperature[bear_id] = right_arm_data['bear_temperatures'][idx]
                self.bear_voltage[bear_id]     = right_arm_data['bear_voltages'][idx]
        
        if CHAIN_ARM_L in chains:
            left_arm_data = MM.LEFT_ARM_JOINT_STATE.get()
            for idx, joint_id in enumerate(ARM_L_JOINT_ID_LIST):
                self.joint_position[joint_id] = left_arm_data['joint_positions'][idx]
                self.joint_velocity[joint_id] = left_arm_data['joint_velocities'][idx]
                self.joint_torque[joint_id]   = left_arm_data['joint_torques'][idx]
            for idx, bear_id in enumerate(ARM_L_BEAR_ID_LIST):
                self.bear_temperature[bear_id] = left_arm_data['bear_temperatures'][idx]
                self.bear_voltage[bear_id]     = left_arm_data['bear_voltages'][idx]
        
        if CHAIN_HEAD in chains:
            head_data = MM.HEAD_JOINT_STATE.get()
            for idx, joint_id in enumerate(HEAD_JOINT_ID_LIST):
                self.joint_position[joint_id] = head_data['joint_positions'][idx]
                self.joint_velocity[joint_id] = head_data['joint_velocities'][idx]
                self.joint_torque[joint_id]   = head_data['joint_torques'][idx]
            for idx, bear_id in enumerate(HEAD_BEAR_ID_LIST):
                self.bear_temperature[bear_id] = head_data['bear_temperatures'][idx]
                self.bear_voltage[bear_id]     = head_data['bear_voltages'][idx]

        if CHAIN_HAND_R in chains:
            right_hand_data = MM.RIGHT_HAND_JOINT_STATE.get()
            for idx, joint_id in enumerate(HAND_R_JOINT_ID_LIST):
                self.joint_position[joint_id] = right_hand_data['joint_positions'][idx]
                self.joint_velocity[joint_id] = right_hand_data['joint_velocities'][idx]
                self.joint_torque[joint_id]   = right_hand_data['joint_torques'][idx]
            for idx, dxl_id in enumerate(HAND_R_DXL_ID_LIST):
                self.dxl_temperature[dxl_id] = right_hand_data['dxl_temperatures'][idx]
                self.dxl_voltage[dxl_id]     = right_hand_data['dxl_voltages'][idx]

        if CHAIN_HAND_L in chains:
            left_hand_data = MM.LEFT_HAND_JOINT_STATE.get()
            for idx, joint_id in enumerate(HAND_L_JOINT_ID_LIST):
                self.joint_position[joint_id] = left_hand_data['joint_positions'][idx]
                self.joint_velocity[joint_id] = left_hand_data['joint_velocities'][idx]
                self.joint_torque[joint_id]   = left_hand_data['joint_torques'][idx]
            for idx, dxl_id in enumerate(HAND_L_DXL_ID_LIST):
                self.dxl_temperature[dxl_id] = left_hand_data['dxl_temperatures'][idx]
                self.dxl_voltage[dxl_id]     = left_hand_data['dxl_voltages'][idx]

    def update_joint_commands(self, chains):
        """
        Update joint commands in shared memory
        """
        if CHAIN_LEG_R in chains:
            commands = {'goal_joint_positions':    np.array([self.goal_joint_position[joint_id]   for joint_id in LEG_R_JOINT_ID_LIST]),
                        'goal_joint_velocities':   np.array([self.goal_joint_velocity[joint_id]   for joint_id in LEG_R_JOINT_ID_LIST]),
                        'goal_joint_torques':      np.array([self.goal_joint_torque[joint_id]     for joint_id in LEG_R_JOINT_ID_LIST]),
                        'bear_enable_statuses':    np.array([self.bear_enable_status[bear_id]     for bear_id  in LEG_R_BEAR_ID_LIST]),
                        'bear_operating_modes':    np.array([self.bear_operating_mode[bear_id]    for bear_id  in LEG_R_BEAR_ID_LIST]),
                        'bear_proportional_gains': np.array([self.bear_proportional_gain[bear_id] for bear_id  in LEG_R_BEAR_ID_LIST]),
                        'bear_derivative_gains':   np.array([self.bear_derivative_gain[bear_id]   for bear_id  in LEG_R_BEAR_ID_LIST])}
            MM.RIGHT_LEG_JOINT_COMMAND.set(commands)

        if CHAIN_LEG_L in chains:
            commands = {'goal_joint_positions':    np.array([self.goal_joint_position[joint_id]   for joint_id in LEG_L_JOINT_ID_LIST]),
                        'goal_joint_velocities':   np.array([self.goal_joint_velocity[joint_id]   for joint_id in LEG_L_JOINT_ID_LIST]),
                        'goal_joint_torques':      np.array([self.goal_joint_torque[joint_id]     for joint_id in LEG_L_JOINT_ID_LIST]),
                        'bear_enable_statuses':    np.array([self.bear_enable_status[bear_id]     for bear_id  in LEG_L_BEAR_ID_LIST]),
                        'bear_operating_modes':    np.array([self.bear_operating_mode[bear_id]    for bear_id  in LEG_L_BEAR_ID_LIST]),
                        'bear_proportional_gains': np.array([self.bear_proportional_gain[bear_id] for bear_id  in LEG_L_BEAR_ID_LIST]),
                        'bear_derivative_gains':   np.array([self.bear_derivative_gain[bear_id]   for bear_id  in LEG_L_BEAR_ID_LIST])}
            MM.LEFT_LEG_JOINT_COMMAND.set(commands)

        if CHAIN_ARM_R in chains:
            commands = {'goal_joint_positions':    np.array([self.goal_joint_position[joint_id]   for joint_id in ARM_R_JOINT_ID_LIST]),
                        'goal_joint_velocities':   np.array([self.goal_joint_velocity[joint_id]   for joint_id in ARM_R_JOINT_ID_LIST]),
                        'goal_joint_torques':      np.array([self.goal_joint_torque[joint_id]     for joint_id in ARM_R_JOINT_ID_LIST]),
                        'bear_enable_statuses':    np.array([self.bear_enable_status[bear_id]     for bear_id  in ARM_R_BEAR_ID_LIST]),
                        'bear_operating_modes':    np.array([self.bear_operating_mode[bear_id]    for bear_id  in ARM_R_BEAR_ID_LIST]),
                        'bear_proportional_gains': np.array([self.bear_proportional_gain[bear_id] for bear_id  in ARM_R_BEAR_ID_LIST]),
                        'bear_derivative_gains':   np.array([self.bear_derivative_gain[bear_id]   for bear_id  in ARM_R_BEAR_ID_LIST])}
            MM.RIGHT_ARM_JOINT_COMMAND.set(commands)

        if CHAIN_ARM_L in chains:
            commands = {'goal_joint_positions':    np.array([self.goal_joint_position[joint_id]   for joint_id in ARM_L_JOINT_ID_LIST]),
                        'goal_joint_velocities':   np.array([self.goal_joint_velocity[joint_id]   for joint_id in ARM_L_JOINT_ID_LIST]),
                        'goal_joint_torques':      np.array([self.goal_joint_torque[joint_id]     for joint_id in ARM_L_JOINT_ID_LIST]),
                        'bear_enable_statuses':    np.array([self.bear_enable_status[bear_id]     for bear_id  in ARM_L_BEAR_ID_LIST]),
                        'bear_operating_modes':    np.array([self.bear_operating_mode[bear_id]    for bear_id  in ARM_L_BEAR_ID_LIST]),
                        'bear_proportional_gains': np.array([self.bear_proportional_gain[bear_id] for bear_id  in ARM_L_BEAR_ID_LIST]),
                        'bear_derivative_gains':   np.array([self.bear_derivative_gain[bear_id]   for bear_id  in ARM_L_BEAR_ID_LIST])}
            MM.LEFT_ARM_JOINT_COMMAND.set(commands)

        if CHAIN_HEAD in chains:
            commands = {'goal_joint_positions':    np.array([self.goal_joint_position[joint_id]   for joint_id in HEAD_JOINT_ID_LIST]),
                        'goal_joint_velocities':   np.array([self.goal_joint_velocity[joint_id]   for joint_id in HEAD_JOINT_ID_LIST]),
                        'goal_joint_torques':      np.array([self.goal_joint_torque[joint_id]     for joint_id in HEAD_JOINT_ID_LIST]),
                        'bear_enable_statuses':    np.array([self.bear_enable_status[bear_id]     for bear_id  in HEAD_BEAR_ID_LIST]),
                        'bear_operating_modes':    np.array([self.bear_operating_mode[bear_id]    for bear_id  in HEAD_BEAR_ID_LIST]),
                        'bear_proportional_gains': np.array([self.bear_proportional_gain[bear_id] for bear_id  in HEAD_BEAR_ID_LIST]),
                        'bear_derivative_gains':   np.array([self.bear_derivative_gain[bear_id]   for bear_id  in HEAD_BEAR_ID_LIST])}
            MM.HEAD_JOINT_COMMAND.set(commands)

        if CHAIN_HAND_R in chains:
            commands = {'goal_joint_positions':   np.array([self.goal_joint_position[joint_id] for joint_id in HAND_R_JOINT_ID_LIST]),
                        'goal_joint_velocities':  np.array([self.goal_joint_velocity[joint_id] for joint_id in HAND_R_JOINT_ID_LIST]),
                        'goal_joint_torques':     np.array([self.goal_joint_torque[joint_id]   for joint_id in HAND_R_JOINT_ID_LIST]),
                        'dxl_enable_statuses':    np.array([self.dxl_enable_status[dxl_id]     for dxl_id   in HAND_R_DXL_ID_LIST]),
                        'dxl_operating_modes':    np.array([self.dxl_operating_mode[dxl_id]    for dxl_id   in HAND_R_DXL_ID_LIST]),
                        'dxl_proportional_gains': np.array([self.dxl_proportional_gain[dxl_id] for dxl_id   in HAND_R_DXL_ID_LIST]),
                        'dxl_derivative_gains':   np.array([self.dxl_derivative_gain[dxl_id]   for dxl_id   in HAND_R_DXL_ID_LIST])}
            MM.RIGHT_HAND_JOINT_COMMAND.set(commands)

        if CHAIN_HAND_L in chains:
            commands = {'goal_joint_positions':   np.array([self.goal_joint_position[joint_id] for joint_id in HAND_L_JOINT_ID_LIST]),
                        'goal_joint_velocities':  np.array([self.goal_joint_velocity[joint_id] for joint_id in HAND_L_JOINT_ID_LIST]),
                        'goal_joint_torques':     np.array([self.goal_joint_torque[joint_id]   for joint_id in HAND_L_JOINT_ID_LIST]),
                        'dxl_enable_statuses':    np.array([self.dxl_enable_status[dxl_id]     for dxl_id   in HAND_L_DXL_ID_LIST]),
                        'dxl_operating_modes':    np.array([self.dxl_operating_mode[dxl_id]    for dxl_id   in HAND_L_DXL_ID_LIST]),
                        'dxl_proportional_gains': np.array([self.dxl_proportional_gain[dxl_id] for dxl_id   in HAND_L_DXL_ID_LIST]),
                        'dxl_derivative_gains':   np.array([self.dxl_derivative_gain[dxl_id]   for dxl_id   in HAND_L_DXL_ID_LIST])}
            MM.LEFT_HAND_JOINT_COMMAND.set(commands)

    def set_joint_commands(self, chains, enable='disable', mode='torque', reset=True):
        """
        Set joint commands
        """
        bear_id_list  = []
        dxl_id_list   = []
        # joint_id_list = []
        if CHAIN_LEG_R in chains:
            bear_id_list  += LEG_R_BEAR_ID_LIST
            # joint_id_list += LEG_R_JOINT_ID_LIST
        if CHAIN_LEG_L in chains:
            bear_id_list  += LEG_L_BEAR_ID_LIST
            # joint_id_list += LEG_L_JOINT_ID_LIST
        if CHAIN_ARM_R in chains:
            bear_id_list  += ARM_R_BEAR_ID_LIST
            # joint_id_list += ARM_R_JOINT_ID_LIST
        if CHAIN_ARM_L in chains:
            bear_id_list  += ARM_L_BEAR_ID_LIST
            # joint_id_list += ARM_L_JOINT_ID_LIST
        if CHAIN_HEAD in chains:
            bear_id_list  += HEAD_BEAR_ID_LIST
            # joint_id_list += HEAD_JOINT_ID_LIST
        if CHAIN_HAND_R in chains:
            dxl_id_list   += HAND_R_DXL_ID_LIST
            # joint_id_list += HAND_R_JOINT_ID_LIST
        if CHAIN_HAND_L in chains:
            dxl_id_list   += HAND_L_DXL_ID_LIST
            # joint_id_list += HAND_L_JOINT_ID_LIST

        if bear_id_list:
            bear_enable = self.bear_enable_status[enable]
            bear_mode   = self.bear_operating_mode[mode]
            # bear_mode   = self.bear_operating_mode['force'] if mode == 'position' else self.bear_operating_mode[mode]
            for bear_id in bear_id_list:
                self.bear_enable_status[bear_id]  = bear_enable
                self.bear_operating_mode[bear_id] = bear_mode

        if dxl_id_list:
            dxl_enable = self.dxl_enable_status[enable]
            dxl_mode   = self.dxl_operating_mode[mode]
            for dxl_id in dxl_id_list:
                self.dxl_enable_status[dxl_id]  = dxl_enable
                self.dxl_operating_mode[dxl_id] = dxl_mode

        # if reset:
        #     for joint_id in joint_id_list:
        #         self.goal_joint_velocity[joint_id] = 0
        #         self.goal_joint_torque[joint_id]   = 0

        self.update_joint_commands(chains)

    def set_joint_positions(self, chains):
        """
        Set joint positions in position mode
        """
        self.set_joint_commands(chains, enable='enable', mode='position', reset=True)

    def set_joint_states(self, chains):
        """
        Set joint states in BEAR force mode
        """
        self.set_joint_commands(chains, enable='enable', mode='force', reset=False)

    def disable_bears(self, chains):
        """
        Set BEAR enable status to 'disable'
        """
        self.set_joint_commands(chains, enable='disable', mode='torque', reset=True)

    def damping_bears(self, chains):
        """
        Set BEAR enable status to 'damping'
        """
        self.set_joint_commands(chains, enable='damping', mode='torque', reset=True)

    def disable_dxls(self, chains):
        """
        Set DXL enable status to 'disable'
        """
        self.set_joint_commands(chains, enable='disable', mode='position', reset=True)
    
    def calculate_foot_FK(self, leg, location, q):
        """
        Calculate leg forward kinematics
        """
        if leg == 'right':
            leg = +1
        elif leg == 'left':
            leg = -1

        if location == 'ankle':
            option = 0
        elif location == 'foot':
            option = 1
        elif location == 'inner toe':
            option = 2
        elif location == 'outer toe':
            option = 3
        elif location == 'heel':
            option = 4

        return KIN.foot_FK(leg, option, q)
    
    def solve_foot_IK(self, leg, location, R, p, q0=None):
        """
        Solve leg inverse kinematics
        """
        if leg == 'right':
            leg = +1
            joint_id_list = LEG_R_JOINT_ID_LIST
        elif leg == 'left':
            leg = -1
            joint_id_list = LEG_L_JOINT_ID_LIST

        if location == 'ankle':
            option = 0
        elif location == 'foot':
            option = 1
        elif location == 'inner toe':
            option = 2
        elif location == 'outer toe':
            option = 3
        elif location == 'heel':
            option = 4

        if q0 is None:
            q0 = np.zeros(6)

        q = KIN.foot_IK(leg, option, R, p, q0)
        
        joint_limit_check_list = [True]  * 4
        orientation_check_list = [False] * 4
        position_check_list    = [False] * 4
        joint_difference_list  = [0]     * 4
        for idx in range(4):
            R1, p1 = self.calculate_foot_FK(leg, location, q[idx, :])
            joint_difference_list[idx] = np.linalg.norm(q[idx, :] - q0)

            if np.linalg.norm(R1 - R) < 1e-3:
                orientation_check_list[idx] = True
                
                for jdx, joint_id in enumerate(joint_id_list):
                    if q[idx, jdx] < JOINT[JOINT_POSITION_LIMIT][joint_id][0]:
                        q[idx, jdx] = JOINT[JOINT_POSITION_LIMIT][joint_id][0]
                        joint_limit_check_list[idx] = False
                    if q[idx, jdx] > JOINT[JOINT_POSITION_LIMIT][joint_id][1]:
                        q[idx, jdx] = JOINT[JOINT_POSITION_LIMIT][joint_id][1]
                        joint_limit_check_list[idx] = False

                if np.linalg.norm(p1 - p) < 1e-3:
                    position_check_list[idx] = True
                    if joint_limit_check_list[idx]:
                        return q[idx, :]
                              
        for idx in range(4):
            if joint_difference_list[idx] < 1:
                if orientation_check_list[idx] and position_check_list[idx]:
                    # cprint("IK Exceeds Joint Limit!", 'yellow')
                    return q[idx, :]
                elif orientation_check_list[idx]:
                    cprint("IK Out of Workspace!", 'yellow')
                    return q[idx, :]
        
        # cprint("IK Infeasible!", 'yellow')
        return q0
    
    def thread_issue(self, thread_name=None):
        """
        Check thread status
        """
        self.update_thread_states()

        error = False

        if self.thread_state['simulation'][0] == 2:
            cprint("Simulation Thread Error! Terminate Now!", 'red')
            error = True
        
        if self.thread_state['bear_right_leg'][0] == 2:
            cprint("BEAR Right Leg Thread Error! Terminate Now!", 'red')
            error = True
            if thread_name == 'bear_right_leg':
                raise THREAD_STOP
        elif self.thread_state['bear_right_leg'][0] == 3:
            cprint("BEAR Right Leg Error! Terminate Now!", 'red')
            error = True
            if thread_name == 'bear_right_leg':
                raise BEAR_ERROR
        elif self.thread_state['bear_right_leg'][0] == 4:
            cprint("BEAR Right Leg ESTOP! Terminate Now!", 'red')
            error = True
            if thread_name == 'bear_right_leg':
                raise BEAR_ESTOP

        if self.thread_state['bear_left_leg'][0] == 2:
            cprint("BEAR Left Leg Thread Error! Terminate Now!", 'red')
            error = True
            if thread_name == 'bear_left_leg':
                raise THREAD_STOP
        elif self.thread_state['bear_left_leg'][0] == 3:
            cprint("BEAR Left Leg Error! Terminate Now!", 'red')
            error = True
            if thread_name == 'bear_left_leg':
                raise BEAR_ERROR
        elif self.thread_state['bear_left_leg'][0] == 4:
            cprint("BEAR Left Leg ESTOP! Terminate Now!", 'red')
            error = True
            if thread_name == 'bear_left_leg':
                raise BEAR_ESTOP
        
        # if self.thread_state['bear_right_arm'][0] == 2:
        #     cprint("BEAR Right Arm Thread Error! Terminate Now!", 'red')
        #     # error = True
        #     if thread_name == 'bear_right_arm':
        #         raise THREAD_STOP
        # elif self.thread_state['bear_right_arm'][0] == 3:
        #     cprint("BEAR Right Arm Error! Terminate Now!", 'red')
        #     # error = True
        #     if thread_name == 'bear_right_arm':
        #         raise BEAR_ERROR
        # elif self.thread_state['bear_right_arm'][0] == 4:
        #     cprint("BEAR Right Arm ESTOP! Terminate Now!", 'red')
        #     # error = True
        #     if thread_name == 'bear_right_arm':
        #         raise BEAR_ESTOP
        
        # if self.thread_state['bear_left_arm'][0] == 2:
        #     cprint("BEAR Left Arm Thread Error! Terminate Now!", 'red')
        #     # error = True
        #     if thread_name == 'bear_left_arm':
        #         raise THREAD_STOP
        # elif self.thread_state['bear_left_arm'][0] == 3:
        #     cprint("BEAR Left Arm Error! Terminate Now!", 'red')
        #     # error = True
        #     if thread_name == 'bear_left_arm':
        #         raise BEAR_ERROR
        # elif self.thread_state['bear_left_arm'][0] == 4:
        #     cprint("BEAR Left Arm ESTOP! Terminate Now!", 'red')
        #     # error = True
        #     if thread_name == 'bear_left_arm':
        #         raise BEAR_ESTOP
            
        # if self.thread_state['bear_head'][0] == 2:
        #     cprint("BEAR Head Thread Error! Terminate Now!", 'red')
        #     # error = True
        #     if thread_name == 'bear_head':
        #         raise THREAD_STOP
        # elif self.thread_state['bear_head'][0] == 3:
        #     cprint("BEAR Head Error! Terminate Now!", 'red')
        #     # error = True
        #     if thread_name == 'bear_head':
        #         raise BEAR_ERROR
        # elif self.thread_state['bear_head'][0] == 4:
        #     cprint("BEAR Head ESTOP! Terminate Now!", 'red')
        #     # error = True
        #     if thread_name == 'bear_head':
        #         raise BEAR_ESTOP

        # if self.thread_state['dxl_right_hand'][0] == 2:
        #     cprint("DXL Right Hand Thread Error! Terminate Now!", 'red')
        #     error = True
        #     if thread_name == 'dxl_right_hand':
        #         raise THREAD_STOP
        # elif self.thread_state['dxl_right_hand'][0] == 3:
        #     cprint("DXL Right Hand Error! Terminate Now!", 'red')
        #     error = True
        #     if thread_name == 'dxl_right_hand':
        #         raise DXL_ERROR
            
        # if self.thread_state['dxl_left_hand'][0] == 2:
        #     cprint("DXL Left Hand Thread Error! Terminate Now!", 'red')
        #     error = True
        #     if thread_name == 'dxl_left_hand':
        #         raise THREAD_STOP
        # elif self.thread_state['dxl_left_hand'][0] == 3:
        #     cprint("DXL Left Hand Error! Terminate Now!", 'red')
        #     error = True
        #     if thread_name == 'dxl_left_hand':
        #         raise DXL_ERROR
            
        if self.thread_state['sense'][0] == 2:
            cprint("Sense Thread Error! Terminate Now!", 'red')
            error = True
        
        if self.thread_state['estimation'][0] == 2:
            cprint("Estimation Thread Error! Terminate Now!", 'red')
            error = True
        
        if self.thread_state['low_level'][0] == 2:
            cprint("Low-Level Thread Error! Terminate Now!", 'red')
            error = True
        
        if self.thread_state['high_level'][0] == 2:
            cprint("High-Level Thread Error! Terminate Now!", 'red')
            error = True
        
        # if self.thread_state['top_level'][0] == 2:
        #     cprint("Top-Level Thread Error! Terminate Now!", 'red')
        #     error = True
        
        # if self.thread_state['posture'][0] == 2:
        #     cprint("Posture Thread Error! Terminate Now!", 'red')
        #     error = True

        if thread_name is not None:
            self.update_thread_commands()
            if self.thread_command[thread_name][0] == 2:
                cprint("Thread Command Received! Terminate Now!", 'red')
                error = True
        
        return error
    
    def get_time(self):
        return self.get_system_time() - self.start_time
    
    def delay_time(self, t):
        t0 = self.get_system_time()
        while self.get_system_time() - t0 < t:
            pass

    def get_system_time(self):
        if self.simulation:
            simulation_data = MM.SIMULATION_STATE.get()
            return simulation_data['time'][0]
        else:
            return time.perf_counter()
            # return time.time()
    
    @staticmethod
    def stop_threading():
        raise THREAD_STOP
