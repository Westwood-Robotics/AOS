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
Script for communication with Gazebo
'''

import time
import numpy as np
import Setting.robot_data as RDS
import Startup.memory_manager as MM
import Library.MATH_FUNCTION.math_function as MF
import Library.ROBOT_MODEL.THEMIS_dynamics as DYN
import Library.ROBOT_MODEL.THEMIS_kinematics as KIN
from Play.config import *
from termcolor import cprint
from Startup.reset_memory import *
from Setting.Macros.model_macros import *
from Setting.Macros.constant_macros import *
from Setting.Macros.locomotion_macros import *
from Library.THEMIS_GYM.GAZEBO_INTERFACE import Manager as gazint


class GazeboSimulator:
    def __init__(self, robot):
        self.robot = robot

        # robot info
        self.num_legs = 2
        self.num_joints_per_leg = 6
        self.num_arms = 2
        self.num_joints_per_arms = 7
        self.num_joints_of_head = 2
        self.num_joints = self.num_legs * self.num_joints_per_leg + self.num_arms * self.num_joints_per_arms + self.num_joints_of_head
        self.num_contact_sensors = 8
        
        self.leg_p_gains = [1000., 1000., 1000., 1000., 500., 100.]
        self.leg_i_gains = [   0.,    0.,    0.,    0.,   0.,   0.]
        self.leg_d_gains = [  30.,   30.,   30.,   30.,   1.,  0.1]

        self.arm_p_gains = [100., 100., 100., 100.,  10.,  1.,   1.]
        self.arm_i_gains = [  0.,   0.,   0.,   0.,   0.,  0.,   0.]
        self.arm_d_gains = [ 10.,  10.,   1.,   1.,  0.1, 0.1, 0.00]

        self.head_p_gains = [50., 50.]
        self.head_i_gains = [ 0.,  0.]
        self.head_d_gains = [ 1.,  1.]

        self.p_gains = self.leg_p_gains * 2 + self.arm_p_gains * 2 + self.head_p_gains  # the joint order matches the urdf file
        self.i_gains = self.leg_i_gains * 2 + self.arm_i_gains * 2 + self.head_i_gains
        self.d_gains = self.leg_d_gains * 2 + self.arm_d_gains * 2 + self.head_d_gains

        self.q   = np.zeros(self.num_joints)
        self.dq  = np.zeros(self.num_joints)
        self.tau = np.zeros(self.num_joints)

        self.k_wc = np.zeros(2)
        self.foot_contact_state = np.zeros(2)
        self.foot_contact_force = np.zeros((2, 3))
        self.accel = np.zeros(3)
        self.omega = np.zeros(3)

        self.leg_r_enable = False
        self.leg_l_enable = False
        self.arm_r_enable = False
        self.arm_l_enable = False
        self.head_enable  = False
        
        # simulator info
        self.simulator = None
        self.simulation_frequency = 1000  # Hz
        self.simulation_modes = {'torque': 0, 'position': 2}
        self.simulation_mode = self.simulation_modes['position']

        # initialize gazebo
        self.initialize_simulator()
        
    def initialize_simulator(self):
        self.simulator = gazint.GazeboInterfaceManager(robot_name='THEMIS', num_joints=self.num_joints, num_contact_sensors=self.num_contact_sensors)
        self.simulator.set_step_size(1. / self.simulation_frequency)
        self.simulator.set_operating_mode(self.simulation_mode)
        self.simulator.set_all_position_pid_gains(self.p_gains, self.i_gains, self.d_gains)

        # leg pose
        p_ba_r = np.array([-0.11, -0.11, -1.05])
        p_ba_l = np.array([-0.11, +0.11, -1.05])
        R_ba_r = np.eye(3)
        R_ba_l = np.eye(3)
        q_leg_r = KIN.foot_IK(+1, 0, R_ba_r, p_ba_r, np.zeros(6))[0, :]
        q_leg_l = KIN.foot_IK(-1, 0, R_ba_l, p_ba_l, np.zeros(6))[0, :]
        
        # for sitting
        # q_leg_r = np.array([0., 0., +1.4, -1.85, +0.55, 0.])
        # q_leg_l = np.array([0., 0., +1.6, -1.85, +0.55, 0.])

        # arm pose
        q_arm_r = np.array([-0.20, +1.40, +1.57, +0.40, -0.00, -0.00, -1.50])
        q_arm_l = np.array([-0.20, -1.40, -1.57, -0.40, +0.00, +0.00, +1.50])

        # head pose
        q_head = np.array([0.0, 0.0])

        # initial pose
        self.initial_pose = np.concatenate((KIN.joint2motor(1, +1, 0, q_leg_r), 
                                            KIN.joint2motor(1, -1, 0, q_leg_l), 
                                            KIN.joint2motor(1, +2, 0, q_arm_r), 
                                            KIN.joint2motor(1, -2, 0, q_arm_l), 
                                            KIN.joint2motor(1,  0, 0, q_head)))
        self.simulator.reset_simulation(initial_pose=self.initial_pose.tolist())

        self.goal_positions = np.array(self.initial_pose)
        self.goal_torques   = np.zeros(self.num_joints)
        
        print('Gazebo Initialization Completed!')

    def set_joint_command(self, right_leg_command=None, left_leg_command=None, right_arm_command=None, left_arm_command=None, head_command=None):
        """
        Send joint command to the simulator
        """
        self.goal_torques = np.zeros(28)

        if right_leg_command['bear_enable_statuses'][0] == 1:
            self.leg_r_enable = True
            if right_leg_command['bear_operating_modes'][0] == 1:
                self.goal_torques[0:6] = right_leg_command['goal_joint_torques']
            elif right_leg_command['bear_operating_modes'][0] == 2:
                self.goal_torques[0:6] = np.diag(self.leg_p_gains) @ (right_leg_command['goal_joint_positions'] - self.q[0:6]) + np.diag(self.leg_d_gains) @ (right_leg_command['goal_joint_velocities'] - self.dq[0:6])
            elif right_leg_command['bear_operating_modes'][0] == 3:
                self.goal_torques[0:6] = right_leg_command['goal_joint_torques']
                self.goal_torques[0:6] += np.diag(right_leg_command['bear_proportional_gains']) @ (right_leg_command['goal_joint_positions'] - self.q[0:6]) + np.diag(right_leg_command['bear_derivative_gains']) @ (right_leg_command['goal_joint_velocities'] - self.dq[0:6])
                # print(right_leg_command['bear_proportional_gains'])
        else:
            if not self.leg_r_enable:
                self.goal_torques[0:6] = np.diag(self.leg_p_gains) @ (self.goal_positions[0:6] - self.q[0:6]) - np.diag(self.leg_d_gains) @ self.dq[0:6]

        if left_leg_command['bear_enable_statuses'][0] == 1:
            self.leg_l_enable = True
            if left_leg_command['bear_operating_modes'][0] == 1:
                self.goal_torques[6:12] = left_leg_command['goal_joint_torques']
            elif left_leg_command['bear_operating_modes'][0] == 2:
                self.goal_torques[6:12] = np.diag(self.leg_p_gains) @ (left_leg_command['goal_joint_positions'] - self.q[6:12]) + np.diag(self.leg_d_gains) @ (left_leg_command['goal_joint_velocities'] - self.dq[6:12])
            elif left_leg_command['bear_operating_modes'][0] == 3:
                self.goal_torques[6:12] = left_leg_command['goal_joint_torques']
                self.goal_torques[6:12] += np.diag(left_leg_command['bear_proportional_gains']) @ (left_leg_command['goal_joint_positions'] - self.q[6:12]) + np.diag(left_leg_command['bear_derivative_gains']) @ (left_leg_command['goal_joint_velocities'] - self.dq[6:12])
                # print(left_leg_command['bear_proportional_gains'])
        else:
            if not self.leg_l_enable:
                self.goal_torques[6:12] = np.diag(self.leg_p_gains) @ (self.goal_positions[6:12] - self.q[6:12]) - np.diag(self.leg_d_gains) @ self.dq[6:12]

        if right_arm_command['bear_enable_statuses'][0] == 1:
            self.arm_r_enable = True
            if right_arm_command['bear_operating_modes'][0] == 1:
                self.goal_torques[12:19] = right_arm_command['goal_joint_torques']
            elif right_arm_command['bear_operating_modes'][0] == 2:
                self.goal_torques[12:19] = np.diag(self.arm_p_gains) @ (right_arm_command['goal_joint_positions'] - self.q[12:19]) + np.diag(self.arm_d_gains) @ (right_arm_command['goal_joint_velocities'] - self.dq[12:19])
            elif right_arm_command['bear_operating_modes'][0] == 3:
                self.goal_torques[12:19] = right_arm_command['goal_joint_torques']
                self.goal_torques[12:19] += np.diag(right_arm_command['bear_proportional_gains']) @ (right_arm_command['goal_joint_positions'] - self.q[12:19]) + np.diag(right_arm_command['bear_derivative_gains']) @ (right_arm_command['goal_joint_velocities'] - self.dq[12:19])
        else:
            if not self.arm_r_enable:
                self.goal_torques[12:19] = np.diag(self.arm_p_gains) @ (self.goal_positions[12:19] - self.q[12:19]) - np.diag(self.arm_d_gains) @ self.dq[12:19]

        if left_arm_command['bear_enable_statuses'][0] == 1:
            self.arm_l_enable = True
            if left_arm_command['bear_operating_modes'][0] == 1:
                self.goal_torques[19:26] = left_arm_command['goal_joint_torques']
            if left_arm_command['bear_operating_modes'][0] == 2:
                self.goal_torques[19:26] = np.diag(self.arm_p_gains) @ (left_arm_command['goal_joint_positions'] - self.q[19:26]) + np.diag(self.arm_d_gains) @ (left_arm_command['goal_joint_velocities'] - self.dq[19:26])
            elif left_arm_command['bear_operating_modes'][0] == 3:
                self.goal_torques[19:26] = left_arm_command['goal_joint_torques']
                self.goal_torques[19:26] += np.diag(left_arm_command['bear_proportional_gains']) @ (left_arm_command['goal_joint_positions'] - self.q[19:26]) + np.diag(left_arm_command['bear_derivative_gains']) @ (left_arm_command['goal_joint_velocities'] - self.dq[19:26])
        else:
            if not self.arm_l_enable:
                self.goal_torques[19:26] = np.diag(self.arm_p_gains) @ (self.goal_positions[19:26] - self.q[19:26]) - np.diag(self.arm_d_gains) @ self.dq[19:26]

        if head_command['bear_enable_statuses'][0] == 1:
            self.head_enable = True
            if head_command['bear_operating_modes'][0] == 1:
                self.goal_torques[26:28] = head_command['goal_joint_torques']
            elif head_command['bear_operating_modes'][0] == 2:
                self.goal_torques[26:28] = np.diag(self.head_p_gains) @ (head_command['goal_joint_positions'] - self.q[26:28]) + np.diag(self.head_d_gains) @ (head_command['goal_joint_velocities'] - self.dq[26:28])
            elif head_command['bear_operating_modes'][0] == 3:
                self.goal_torques[26:28] = head_command['goal_joint_torques']
                self.goal_torques[26:28] += np.diag(head_command['bear_proportional_gains']) @ (head_command['goal_joint_positions'] - self.q[26:28]) + np.diag(head_command['bear_derivative_gains']) @ (head_command['goal_joint_velocities'] - self.dq[26:28])
        else:
            if not self.head_enable:
                self.goal_torques[26:28] = np.diag(self.head_p_gains) @ (self.goal_positions[26:28] - self.q[26:28]) - np.diag(self.head_d_gains) @ self.dq[26:28]

        if self.leg_r_enable or self.leg_l_enable or self.arm_r_enable or self.arm_l_enable or self.head_enable:
            if self.simulation_mode != self.simulation_modes['torque']:
                self.simulation_mode = self.simulation_modes['torque']
                self.simulator.set_operating_mode(self.simulation_mode)
            
            self.simulator.set_command_torque(self.goal_torques.tolist())

    def update_sensor_info(self):
        """
        Get sensor info and write it to shared memory
        """
        # get sim time
        MM.SIMULATION_STATE.set({'time': np.array([self.simulator.get_current_time()])})

        # get joint states
        q_raw   = self.simulator.get_current_position()
        dq_raw  = self.simulator.get_current_velocity()
        tau_raw = self.simulator.get_current_force()

        q_raw[0:6]     = KIN.motor2joint(1, +1, 0, q_raw[0:6])
        q_raw[6:12]    = KIN.motor2joint(1, -1, 0, q_raw[6:12])
        q_raw[12:19]   = KIN.motor2joint(1, +2, 0, q_raw[12:19])
        q_raw[19:26]   = KIN.motor2joint(1, -2, 0, q_raw[19:26])
        q_raw[26:28]   = KIN.motor2joint(1,  0, 0, q_raw[26:28])

        dq_raw[0:6]    = KIN.motor2joint(1, +1, 1, dq_raw[0:6])
        dq_raw[6:12]   = KIN.motor2joint(1, -1, 1, dq_raw[6:12])
        dq_raw[12:19]  = KIN.motor2joint(1, +2, 1, dq_raw[12:19])
        dq_raw[19:26]  = KIN.motor2joint(1, -2, 1, dq_raw[19:26])
        dq_raw[26:28]  = KIN.motor2joint(1,  0, 1, dq_raw[26:28])

        tau_raw[0:6]   = KIN.motor2joint(1, +1, 2, tau_raw[0:6])
        tau_raw[6:12]  = KIN.motor2joint(1, -1, 2, tau_raw[6:12])
        tau_raw[12:19] = KIN.motor2joint(1, +2, 2, tau_raw[12:19])
        tau_raw[19:26] = KIN.motor2joint(1, -2, 2, tau_raw[19:26])
        tau_raw[26:28] = KIN.motor2joint(1,  0, 2, tau_raw[26:28])

        if ESTIMATION:
            q_raw   += np.random.randn(self.num_joints) * DEG2RAD * 0.01
            dq_raw  += np.random.randn(self.num_joints) * DEG2RAD * 0.01
            tau_raw += np.random.randn(self.num_joints) * 0.01
            alp = 0.1
        else:
            alp = 0.0

        for idx in range(self.num_joints):
            self.q[idx]   = MF.exp_filter(self.q[idx],   q_raw[idx],   alp)
            self.dq[idx]  = MF.exp_filter(self.dq[idx],  dq_raw[idx],  alp)
            self.tau[idx] = MF.exp_filter(self.tau[idx], tau_raw[idx], alp)

        right_leg_data = {'joint_positions':  self.q[0:6],
                          'joint_velocities': self.dq[0:6],
                          'joint_torques':    self.tau[0:6]}
        MM.RIGHT_LEG_JOINT_STATE.set(right_leg_data)

        left_leg_data =  {'joint_positions':  self.q[6:12],
                          'joint_velocities': self.dq[6:12],
                          'joint_torques':    self.tau[6:12]}
        MM.LEFT_LEG_JOINT_STATE.set(left_leg_data)

        right_arm_data = {'joint_positions':  self.q[12:19],
                          'joint_velocities': self.dq[12:19],
                          'joint_torques':    self.tau[12:19]}
        MM.RIGHT_ARM_JOINT_STATE.set(right_arm_data)

        left_arm_data =  {'joint_positions':  self.q[19:26],
                          'joint_velocities': self.dq[19:26],
                          'joint_torques':    self.tau[19:26]}
        MM.LEFT_ARM_JOINT_STATE.set(left_arm_data)

        head_data =      {'joint_positions':  self.q[26:28],
                          'joint_velocities': self.dq[26:28],
                          'joint_torques':    self.tau[26:28]}
        MM.HEAD_JOINT_STATE.set(head_data)
        
        # get sense states
        self.R_wb   = self.simulator.get_body_rot_mat()
        accel_raw   = self.simulator.get_imu_acceleration() + np.array([0., 0., 0.])
        omega_raw   = self.R_wb.T @ self.simulator.get_imu_angular_rate()
        contact_raw = self.simulator.get_foot_contacts()

        if ESTIMATION:
            accel_raw += np.random.randn(3) * 0.01
            omega_raw += np.random.randn(3) * 0.001
            alp = 0.1
        else:
            alp = 0.0

        for idx in range(3):
            self.accel[idx] = MF.exp_filter(self.accel[idx], accel_raw[idx], alp)
            self.omega[idx] = MF.exp_filter(self.omega[idx], omega_raw[idx], alp)

        sense_data = {'imu_acceleration':    self.accel,
                      'imu_angular_rate':    self.omega,
                      'imu_rotation_matrix': self.R_wb,
                      'foot_contact_state':  np.vstack((contact_raw[0:3], contact_raw[4:7]))}
        MM.SENSE_STATE.set(sense_data)
        
    def calculate_robot_model(self, robot):
        """
        Calculate kinematics & dynamics and write it to shared memory
        """
        q_leg_r  = self.q[0:6]
        q_leg_l  = self.q[6:12]
        dq_leg_r = self.dq[0:6]
        dq_leg_l = self.dq[6:12]

        q_arm_r  = self.q[12:19]
        q_arm_l  = self.q[19:26]
        dq_arm_r = self.dq[12:19]
        dq_arm_l = self.dq[19:26]

        q_head  = self.q[26:28]
        dq_head = self.dq[26:28]

        R_wb = self.R_wb
        w_bb = self.omega
        p_wb = self.simulator.get_body_position()
        v_wb = self.simulator.get_body_velocity()
        a_wb = self.simulator.get_body_rot_mat() @ self.accel + np.array([0., 0., GRAVITY_ACCEL])
        v_bb = R_wb.T @ v_wb
        a_bb = R_wb.T @ a_wb
        heading = np.arctan2(R_wb[1, 0], R_wb[0, 0])

        Jw_bb    = np.eye(3, 6)
        dJwdq_bb = np.zeros(3)
        Jv_wb    = np.hstack((np.zeros((3, 3)), R_wb))
        
        dJvdq_wb = R_wb @ MF.hat(w_bb) @ v_bb

        R_ba_r, w_ba_r, p_ba_r, v_ba_r, Jw_ba_r, dJwdq_ba_r, Jv_ba_r, dJvdq_ba_r = KIN.chain_FK(+1, 0, q_leg_r, dq_leg_r)
        R_ba_l, w_ba_l, p_ba_l, v_ba_l, Jw_ba_l, dJwdq_ba_l, Jv_ba_l, dJvdq_ba_l = KIN.chain_FK(-1, 0, q_leg_l, dq_leg_l)
        R_bf_r, w_bf_r, p_bf_r, v_bf_r, Jw_bf_r, dJwdq_bf_r, Jv_bf_r, dJvdq_bf_r = KIN.chain_FK(+1, 1, q_leg_r, dq_leg_r)
        R_bf_l, w_bf_l, p_bf_l, v_bf_l, Jw_bf_l, dJwdq_bf_l, Jv_bf_l, dJvdq_bf_l = KIN.chain_FK(-1, 1, q_leg_l, dq_leg_l)

        R_bh_r, w_bh_r, p_bh_r, v_bh_r, Jw_bh_r, dJwdq_bh_r, Jv_bh_r, dJvdq_bh_r = KIN.chain_FK(+2, 1, q_arm_r, dq_arm_r)
        R_bh_l, w_bh_l, p_bh_l, v_bh_l, Jw_bh_l, dJwdq_bh_l, Jv_bh_l, dJvdq_bh_l = KIN.chain_FK(-2, 1, q_arm_l, dq_arm_l)

        R_bc, w_bc, p_bc, v_bc, Jw_bc, dJwdq_bc, Jv_bc, dJvdq_bc = KIN.chain_FK(0, 1, q_head, dq_head)

        R_wa_r, w_fa_r, p_wa_r, v_wa_r, Jw_fa_r, dJwdq_fa_r, Jv_wa_r, dJvdq_wa_r = KIN.base2world(R_wb, w_bb, p_wb, v_bb, R_ba_r, w_ba_r, p_ba_r, v_ba_r, Jw_ba_r, dJwdq_ba_r, Jv_ba_r, dJvdq_ba_r)
        R_wa_l, w_fa_l, p_wa_l, v_wa_l, Jw_fa_l, dJwdq_fa_l, Jv_wa_l, dJvdq_wa_l = KIN.base2world(R_wb, w_bb, p_wb, v_bb, R_ba_l, w_ba_l, p_ba_l, v_ba_l, Jw_ba_l, dJwdq_ba_l, Jv_ba_l, dJvdq_ba_l)
        R_wf_r, w_ff_r, p_wf_r, v_wf_r, Jw_ff_r, dJwdq_ff_r, Jv_wf_r, dJvdq_wf_r = KIN.base2world(R_wb, w_bb, p_wb, v_bb, R_bf_r, w_bf_r, p_bf_r, v_bf_r, Jw_bf_r, dJwdq_bf_r, Jv_bf_r, dJvdq_bf_r)
        R_wf_l, w_ff_l, p_wf_l, v_wf_l, Jw_ff_l, dJwdq_ff_l, Jv_wf_l, dJvdq_wf_l = KIN.base2world(R_wb, w_bb, p_wb, v_bb, R_bf_l, w_bf_l, p_bf_l, v_bf_l, Jw_bf_l, dJwdq_bf_l, Jv_bf_l, dJvdq_bf_l)

        R_wh_r, w_hh_r, p_wh_r, v_wh_r, Jw_hh_r, dJwdq_hh_r, Jv_wh_r, dJvdq_wh_r = KIN.base2world(R_wb, w_bb, p_wb, v_bb, R_bh_r, w_bh_r, p_bh_r, v_bh_r, Jw_bh_r, dJwdq_bh_r, Jv_bh_r, dJvdq_bh_r)
        R_wh_l, w_hh_l, p_wh_l, v_wh_l, Jw_hh_l, dJwdq_hh_l, Jv_wh_l, dJvdq_wh_l = KIN.base2world(R_wb, w_bb, p_wb, v_bb, R_bh_l, w_bh_l, p_bh_l, v_bh_l, Jw_bh_l, dJwdq_bh_l, Jv_bh_l, dJvdq_bh_l)

        R_wc, w_cc, p_wc, v_wc, Jw_cc, dJwdq_cc, Jv_wc, dJvdq_wc = KIN.base2world(R_wb, w_bb, p_wb, v_bb, R_bc, w_bc, p_bc, v_bc, Jw_bc, dJwdq_bc, Jv_bc, dJvdq_bc)
        

        H, CG, AG, dAGdq, p_wg, v_wg, h = DYN.robot_ID(R_wb, p_wb, w_bb, v_bb,
                                                       q_leg_r, q_leg_l, q_arm_r, q_arm_l, q_head,
                                                       dq_leg_r, dq_leg_l, dq_arm_r, dq_arm_l, dq_head)
        
        # estimate angular momentum
        robot.update_locomotion_planner_states()
        if robot.locomotion_phase == STANCE_DOUBLE:
            p_wcop = 0.5 * (p_wa_r + p_wa_l)
        elif robot.locomotion_phase == STANCE_RIGHT:
            p_wcop = np.copy(p_wa_r)
        elif robot.locomotion_phase == STANCE_LEFT:
            p_wcop = np.copy(p_wa_l)
            
        if robot.locomotion_phase in [STANCE_DOUBLE, STANCE_RIGHT, STANCE_LEFT]:
            alp = 0.1
            vec = p_wg - p_wcop + np.array([0., 0., +0.05])
            k_wc_obs = h[0:3] + ROBOT_MASS * MF.hat(vec) @ v_wg
            self.k_wc[0] = alp * (self.k_wc[0] - ROBOT_MASS * GRAVITY_ACCEL * vec[1] / self.simulation_frequency) + (1 - alp) * k_wc_obs[0]
            self.k_wc[1] = alp * (self.k_wc[1] + ROBOT_MASS * GRAVITY_ACCEL * vec[0] / self.simulation_frequency) + (1 - alp) * k_wc_obs[1]
        
        # estimate foot contact
        val = np.linalg.pinv(np.hstack((np.vstack((Jv_wa_r[:, 6:10].T, np.zeros((3, 4)).T)), np.vstack((np.zeros((3, 4)).T, Jv_wa_l[:, 6:10].T))))) @ (np.hstack((CG[6:10], CG[12:16])) - np.hstack((self.tau[0:4], self.tau[6:10])))
        for idx in range(3):
            self.foot_contact_force[0, idx] = MF.exp_filter(self.foot_contact_force[0, idx], val[idx],   0.1)
            self.foot_contact_force[1, idx] = MF.exp_filter(self.foot_contact_force[1, idx], val[idx+3], 0.1)
        self.foot_contact_state[0] = 1 if self.foot_contact_force[0, 2] > 50 else 0
        self.foot_contact_state[1] = 1 if self.foot_contact_force[1, 2] > 50 else 0

        # save data
        base_data = {}
        base_data['base_position']        = p_wb
        base_data['base_velocity']        = v_wb
        base_data['base_acceleration']    = a_wb
        base_data['base_rotation_matrix'] = R_wb
        base_data['base_heading']         = np.array([heading])
        base_data['base_angular_rate']    = w_bb
        base_data['base_Jw']              = Jw_bb
        base_data['base_dJwdq']           = dJwdq_bb
        base_data['base_Jv']              = Jv_wb
        base_data['base_dJvdq']           = dJvdq_wb
        MM.BASE_STATE.set(base_data)

        com_data = {}
        com_data['com_position']         = p_wg
        com_data['com_velocity']         = v_wg
        com_data['linear_momentum']      = h[3:6]
        com_data['angular_momentum']     = h[0:3]
        com_data['angular_momentum_cop'] = self.k_wc
        com_data['H_matrix']             = H
        com_data['CG_vector']            = CG
        com_data['AG_matrix']            = AG
        com_data['dAGdq_vector']         = dAGdq
        MM.COM_STATE.set(com_data)

        right_foot_data = {}
        right_foot_data['right_foot_rotation_matrix'] = R_wf_r
        right_foot_data['right_foot_angular_rate']    = w_ff_r
        right_foot_data['right_foot_Jw']              = Jw_ff_r
        right_foot_data['right_foot_dJwdq']           = dJwdq_ff_r
        right_foot_data['right_foot_position']        = p_wf_r
        right_foot_data['right_foot_velocity']        = v_wf_r
        right_foot_data['right_foot_Jv']              = Jv_wf_r
        right_foot_data['right_foot_dJvdq']           = dJvdq_wf_r
        right_foot_data['right_foot_contact_state']   = self.foot_contact_state[0]
        right_foot_data['right_foot_contact_force']   = self.foot_contact_force[0, :]
        MM.RIGHT_FOOT_STATE.set(right_foot_data)

        left_foot_data = {}
        left_foot_data['left_foot_rotation_matrix'] = R_wf_l
        left_foot_data['left_foot_angular_rate']    = w_ff_l
        left_foot_data['left_foot_Jw']              = Jw_ff_l
        left_foot_data['left_foot_dJwdq']           = dJwdq_ff_l
        left_foot_data['left_foot_position']        = p_wf_l
        left_foot_data['left_foot_velocity']        = v_wf_l
        left_foot_data['left_foot_Jv']              = Jv_wf_l
        left_foot_data['left_foot_dJvdq']           = dJvdq_wf_l
        left_foot_data['left_foot_contact_state']   = self.foot_contact_state[1]
        left_foot_data['left_foot_contact_force']   = self.foot_contact_force[1, :]
        MM.LEFT_FOOT_STATE.set(left_foot_data)

        right_ankle_data = {}
        right_ankle_data['right_ankle_position'] = p_wa_r
        right_ankle_data['right_ankle_velocity'] = v_wa_r
        right_ankle_data['right_ankle_Jv']       = Jv_wa_r
        right_ankle_data['right_ankle_dJvdq']    = dJvdq_wa_r
        MM.RIGHT_ANKLE_STATE.set(right_ankle_data)

        left_ankle_data = {}
        left_ankle_data['left_ankle_position'] = p_wa_l
        left_ankle_data['left_ankle_velocity'] = v_wa_l
        left_ankle_data['left_ankle_Jv']       = Jv_wa_l
        left_ankle_data['left_ankle_dJvdq']    = dJvdq_wa_l
        MM.LEFT_ANKLE_STATE.set(left_ankle_data)

        right_hand_data = {}
        right_hand_data['right_hand_rotation_matrix'] = R_wh_r
        right_hand_data['right_hand_angular_rate']    = w_hh_r
        right_hand_data['right_hand_Jw']              = Jw_hh_r
        right_hand_data['right_hand_dJwdq']           = dJwdq_hh_r
        right_hand_data['right_hand_position']        = p_wh_r
        right_hand_data['right_hand_velocity']        = v_wh_r
        right_hand_data['right_hand_Jv']              = Jv_wh_r
        right_hand_data['right_hand_dJvdq']           = dJvdq_wh_r
        MM.RIGHT_HAND_STATE.set(right_hand_data)

        left_hand_data = {}
        left_hand_data['left_hand_rotation_matrix'] = R_wh_l
        left_hand_data['left_hand_angular_rate']    = w_hh_l
        left_hand_data['left_hand_Jw']              = Jw_hh_l
        left_hand_data['left_hand_dJwdq']           = dJwdq_hh_l
        left_hand_data['left_hand_position']        = p_wh_l
        left_hand_data['left_hand_velocity']        = v_wh_l
        left_hand_data['left_hand_Jv']              = Jv_wh_l
        left_hand_data['left_hand_dJvdq']           = dJvdq_wh_l
        MM.LEFT_HAND_STATE.set(left_hand_data)

        head_camera_data = {}
        head_camera_data['head_camera_rotation_matrix'] = R_wc
        head_camera_data['head_camera_angular_rate']    = w_cc
        head_camera_data['head_camera_position']        = p_wc
        head_camera_data['head_camera_velocity']        = v_wc
        MM.HEAD_CAMERA_STATE.set(head_camera_data)


def main_loop():
    check_thread_frequency = 10
    check_thread_duration  = 1. / check_thread_frequency

    print("====== THEMIS Simulation Thread is running... ======")

    last_check_thread_time = time.perf_counter()

    thread_check = False
    
    t0 = time.perf_counter()
    while True:
        loop_start_time = time.perf_counter()

        # check threading issue
        last_check_thread_elapse = loop_start_time - last_check_thread_time
        if last_check_thread_elapse > check_thread_duration:
            last_check_thread_time = loop_start_time
            if Themis.thread_issue():
                Themis.stop_threading()

        if not thread_check and loop_start_time - t0 > 1:
            cprint("THREAD IS DOING GREAT!", 'green')
            MM.THREAD_STATE.set({'simulation': np.array([1])}, opt='update')  # thread is running
            thread_check = True

        gs.update_sensor_info()
        if not ESTIMATION:
            gs.calculate_robot_model(Themis)

        right_leg_command = MM.RIGHT_LEG_JOINT_COMMAND.get()
        left_leg_command  = MM.LEFT_LEG_JOINT_COMMAND.get()
        right_arm_command = MM.RIGHT_ARM_JOINT_COMMAND.get()
        left_arm_command  = MM.LEFT_ARM_JOINT_COMMAND.get()
        head_command      = MM.HEAD_JOINT_COMMAND.get()
        gs.set_joint_command(right_leg_command, left_leg_command, right_arm_command, left_arm_command, head_command)

        gs.simulator.step_simulation()
        # time.sleep(0.01)


if __name__ == '__main__':
    Themis = RDS.RobotDataManager()
    gs = GazeboSimulator(robot=Themis)
    
    try:
        main_loop()
    except RDS.THREAD_STOP:
        cprint("THREAD STOPPED PEACEFULLY!", 'light_grey')
        MM.THREAD_STATE.set({'simulation': np.array([0])}, opt='update')  # thread is stopped
    except (Exception, KeyboardInterrupt) as error:
        print(error)
        cprint("THREAD IN ERROR! TERMINATE NOW!", 'red')
        MM.THREAD_STATE.set({'simulation': np.array([2])}, opt='update')  # thread in error
    finally:
        MM.SIMULATION_STATE.set({'time': np.array([gs.simulator.get_current_time() + 1.])})