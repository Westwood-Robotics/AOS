#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 20, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Script for THEMIS state estimation and model computation
'''

import time
import numpy as np
import Setting.robot_data as RDS
import Startup.memory_manager as MM
import Library.MATH_FUNCTION.math_function as MF
from Play.config import *
from termcolor import cprint
from Setting.Macros.model_macros import *
from Setting.Macros.locomotion_macros import *
from Library.ROBOT_MODEL import THEMIS_dynamics as DYN
from Library.ROBOT_MODEL import THEMIS_kinematics as KIN
from Library.STATE_ESTIMATION import THEMIS_estimation as EST
from Library.STATE_ESTIMATION import THEMIS_estimation_sim as EST_SIM


class StateEstimation:
    def __init__(self, robot=RDS.RobotDataManager(), frequency=500):
        # misc
        self.robot = robot
        self.dt    = 1. / frequency
        
        self.gcc = GRAVITY_ACCEL if HARDWARE else -GRAVITY_ACCEL

        # initial guess
        self.p_wb   = np.array([0.00, 0.00, 1.00])   # base position          - in world frame
        self.v_wb   = np.array([0.00, 0.00, 0.00])   # base velocity          - in world frame
        self.a_wb   = np.array([0.00, 0.00, 0.00])   # base acceleration      - in world frame
        self.ba_bb  = np.array([0.00, 0.00, 0.00])   # imu accelerometer bias - in  base frame
        self.bw_bb  = np.array([0.00, 0.00, 0.00])   # imu gyroscope bias     - in  base frame
        self.R_wb   = np.eye(3)                      # base orientation       - in world frame
        self.v_bb   = np.array([0.00, 0.00, 0.00])   # base velocity          - in  base frame
        self.a_bb   = np.array([0.00, 0.00, 0.00])   # base acceleration      - in  base frame
        self.w_bb   = np.array([0.00, 0.00, 0.00])   # base angular velocity  - in  base frame
        
        self.p_wa_r = np.array([0.00, -0.12, 0.05])  # right ankle position  - in world frame
        self.p_wf_r = np.array([0.00, -0.12, 0.00])  # right foot  position  - in world frame
        
        self.c_wa_r = np.array([0.00, -0.12, 0.05])  # right ankle position if in contact
        self.c_wf_r = np.array([0.00, -0.12, 0.00])  # right foot  position if in contact
        
        self.p_wa_l = np.array([0.00, +0.12, 0.05])  # left  ankle position  - in world frame
        self.p_wf_l = np.array([0.00, +0.12, 0.00])  # left  foot  position  - in world frame
        
        self.c_wa_l = np.array([0.00, +0.12, 0.05])  # left  ankle position if in contact
        self.c_wf_l = np.array([0.00, +0.12, 0.00])  # left  foot  position if in contact
        
        self.Po     = np.eye(15) * 1e-2 if ACCEL_BIAS else np.eye(12) * 1e-2  # state covariance matrix

        self.k_wc   = np.zeros(2)  # horizontal angular momentum about CoP - in world frame
        self.p_wc   = np.zeros(3)  # center of pressure                    - in world frame

        self.heading = 0.0

        # foot contacts
        self.foot_contact_count = np.zeros(2)
        self.foot_contact_force = np.zeros((2, 3))
        self.foot_contact_state = np.zeros(2)  # 0/1
        
        # initialization
        if HARDWARE:
            self.check_sensor()

    def update(self):
        """
        Update estimator thread
        """
        self.update_sensor()
        self.update_estimator()
        self.update_model()
        self.set_states()

    def update_sensor(self):
        """
        Update sensor info
        """
        self.robot.update_locomotion_planner_states()
        self.robot.update_sense_states()
        self.robot.update_joint_states(chains=ROBOT_CHAIN_LIST)

        # joints
        self.q_leg_r   = np.array([self.robot.joint_position[joint] for joint in LEG_R_JOINT_ID_LIST])
        self.q_leg_l   = np.array([self.robot.joint_position[joint] for joint in LEG_L_JOINT_ID_LIST])
        self.dq_leg_r  = np.array([self.robot.joint_velocity[joint] for joint in LEG_R_JOINT_ID_LIST])
        self.dq_leg_l  = np.array([self.robot.joint_velocity[joint] for joint in LEG_L_JOINT_ID_LIST])
        self.tau_leg_r = np.array([self.robot.joint_torque[joint]   for joint in LEG_R_JOINT_ID_LIST])
        self.tau_leg_l = np.array([self.robot.joint_torque[joint]   for joint in LEG_L_JOINT_ID_LIST])

        self.q_arm_r   = np.array([self.robot.joint_position[joint] for joint in ARM_R_JOINT_ID_LIST])
        self.q_arm_l   = np.array([self.robot.joint_position[joint] for joint in ARM_L_JOINT_ID_LIST])
        self.dq_arm_r  = np.array([self.robot.joint_velocity[joint] for joint in ARM_R_JOINT_ID_LIST])
        self.dq_arm_l  = np.array([self.robot.joint_velocity[joint] for joint in ARM_L_JOINT_ID_LIST])
        # self.tau_arm_r = np.array([self.robot.joint_torque[joint]   for joint in ARM_R_JOINT_ID_LIST])
        # self.tau_arm_l = np.array([self.robot.joint_torque[joint]   for joint in ARM_L_JOINT_ID_LIST])

        self.q_head   = np.array([self.robot.joint_position[joint] for joint in HEAD_JOINT_ID_LIST])
        self.dq_head  = np.array([self.robot.joint_velocity[joint] for joint in HEAD_JOINT_ID_LIST])
        # self.tau_head = np.array([self.robot.joint_torque[joint]   for joint in HEAD_JOINT_ID_LIST])

        # planned foot contacts
        if self.robot.locomotion_phase == STANCE_DOUBLE:
            self.foot_contact_count[0] += 1
            self.foot_contact_count[1] += 1
        elif self.robot.locomotion_phase == STANCE_RIGHT:
            self.foot_contact_count[0] += 1
            self.foot_contact_count[1]  = 0
        elif self.robot.locomotion_phase == STANCE_LEFT:
            self.foot_contact_count[0]  = 0
            self.foot_contact_count[1] += 1
        else:
            self.foot_contact_count[0]  = 0
            self.foot_contact_count[1]  = 0

        # leg FK
        self.R_ba_r, self.w_ba_r, self.p_ba_r, self.v_ba_r, self.Jw_ba_r, self.dJwdq_ba_r, self.Jv_ba_r, self.dJvdq_ba_r = KIN.chain_FK(+1, 0, self.q_leg_r, self.dq_leg_r)
        self.R_ba_l, self.w_ba_l, self.p_ba_l, self.v_ba_l, self.Jw_ba_l, self.dJwdq_ba_l, self.Jv_ba_l, self.dJvdq_ba_l = KIN.chain_FK(-1, 0, self.q_leg_l, self.dq_leg_l)
        self.R_bf_r, self.w_bf_r, self.p_bf_r, self.v_bf_r, self.Jw_bf_r, self.dJwdq_bf_r, self.Jv_bf_r, self.dJvdq_bf_r = KIN.chain_FK(+1, 1, self.q_leg_r, self.dq_leg_r)
        self.R_bf_l, self.w_bf_l, self.p_bf_l, self.v_bf_l, self.Jw_bf_l, self.dJwdq_bf_l, self.Jv_bf_l, self.dJvdq_bf_l = KIN.chain_FK(-1, 1, self.q_leg_l, self.dq_leg_l)

        # arm FK
        self.R_bh_r, self.w_bh_r, self.p_bh_r, self.v_bh_r, self.Jw_bh_r, self.dJwdq_bh_r, self.Jv_bh_r, self.dJvdq_bh_r = KIN.chain_FK(+2, 1, self.q_arm_r, self.dq_arm_r)
        self.R_bh_l, self.w_bh_l, self.p_bh_l, self.v_bh_l, self.Jw_bh_l, self.dJwdq_bh_l, self.Jv_bh_l, self.dJvdq_bh_l = KIN.chain_FK(-2, 1, self.q_arm_l, self.dq_arm_l)

        # head camera FK
        self.R_bc, self.w_bc, self.p_bc, self.v_bc, self.Jw_bc, self.dJwdq_bc, self.Jv_bc, self.dJvdq_bc = KIN.chain_FK(0, 1, self.q_head, self.dq_head)

    def update_estimator(self):
        """
        Update state estimator
        """
        try:
            if HARDWARE:
                self.R_wb, self.w_bb, self.bw_bb, self.p_wb, self.v_wb, self.a_wb, self.ba_bb, self.c_wa_r, self.c_wa_l, self.Po, \
                self.v_bb, self.a_bb, self.heading = EST.run(self.R_wb, self.w_bb, self.bw_bb, self.p_wb, self.v_wb, self.a_wb, self.ba_bb, self.c_wa_r, self.c_wa_l, self.Po,
                                                             self.p_ba_r, self.p_ba_l, self.v_ba_r, self.v_ba_l, self.p_wa_r, self.p_wa_l, self.Jv_ba_r[:, 0:4], self.Jv_ba_l[:, 0:4], self.R_wa_r, self.R_wa_l,
                                                             self.robot.imu_rotation_matrix, self.robot.imu_angular_rate, self.robot.imu_acceleration, self.foot_contact_count, self.gcc)
            else:
                self.R_wb, self.w_bb, self.bw_bb, self.p_wb, self.v_wb, self.a_wb, self.ba_bb, self.c_wa_r, self.c_wa_l, self.Po, \
                self.v_bb, self.a_bb, self.heading = EST_SIM.run(self.R_wb, self.w_bb, self.bw_bb, self.p_wb, self.v_wb, self.a_wb, self.ba_bb, self.c_wa_r, self.c_wa_l, self.Po,
                                                                 self.p_ba_r, self.p_ba_l, self.v_ba_r, self.v_ba_l, self.p_wa_r, self.p_wa_l, self.Jv_ba_r[:, 0:4], self.Jv_ba_l[:, 0:4], self.R_wa_r, self.R_wa_l,
                                                                 self.robot.imu_rotation_matrix, self.robot.imu_angular_rate, self.robot.imu_acceleration, self.foot_contact_count, self.gcc)
        except:
            pass

    def update_model(self):
        """
        Update robot states based on estimation
        """
        # kinematics
        self.R_wa_r, self.w_fa_r, self.p_wa_r, self.v_wa_r, self.Jw_fa_r, self.dJwdq_fa_r, self.Jv_wa_r, self.dJvdq_wa_r = KIN.base2world(self.R_wb, self.w_bb, self.p_wb, self.v_bb, self.R_ba_r, self.w_ba_r, self.p_ba_r, self.v_ba_r, self.Jw_ba_r, self.dJwdq_ba_r, self.Jv_ba_r, self.dJvdq_ba_r)
        self.R_wa_l, self.w_fa_l, self.p_wa_l, self.v_wa_l, self.Jw_fa_l, self.dJwdq_fa_l, self.Jv_wa_l, self.dJvdq_wa_l = KIN.base2world(self.R_wb, self.w_bb, self.p_wb, self.v_bb, self.R_ba_l, self.w_ba_l, self.p_ba_l, self.v_ba_l, self.Jw_ba_l, self.dJwdq_ba_l, self.Jv_ba_l, self.dJvdq_ba_l)
        self.R_wf_r, self.w_ff_r, self.p_wf_r, self.v_wf_r, self.Jw_ff_r, self.dJwdq_ff_r, self.Jv_wf_r, self.dJvdq_wf_r = KIN.base2world(self.R_wb, self.w_bb, self.p_wb, self.v_bb, self.R_bf_r, self.w_bf_r, self.p_bf_r, self.v_bf_r, self.Jw_bf_r, self.dJwdq_bf_r, self.Jv_bf_r, self.dJvdq_bf_r)
        self.R_wf_l, self.w_ff_l, self.p_wf_l, self.v_wf_l, self.Jw_ff_l, self.dJwdq_ff_l, self.Jv_wf_l, self.dJvdq_wf_l = KIN.base2world(self.R_wb, self.w_bb, self.p_wb, self.v_bb, self.R_bf_l, self.w_bf_l, self.p_bf_l, self.v_bf_l, self.Jw_bf_l, self.dJwdq_bf_l, self.Jv_bf_l, self.dJvdq_bf_l)

        self.R_wh_r, self.w_hh_r, self.p_wh_r, self.v_wh_r, self.Jw_hh_r, self.dJwdq_hh_r, self.Jv_wh_r, self.dJvdq_wh_r = KIN.base2world(self.R_wb, self.w_bb, self.p_wb, self.v_bb, self.R_bh_r, self.w_bh_r, self.p_bh_r, self.v_bh_r, self.Jw_bh_r, self.dJwdq_bh_r, self.Jv_bh_r, self.dJvdq_bh_r)
        self.R_wh_l, self.w_hh_l, self.p_wh_l, self.v_wh_l, self.Jw_hh_l, self.dJwdq_hh_l, self.Jv_wh_l, self.dJvdq_wh_l = KIN.base2world(self.R_wb, self.w_bb, self.p_wb, self.v_bb, self.R_bh_l, self.w_bh_l, self.p_bh_l, self.v_bh_l, self.Jw_bh_l, self.dJwdq_bh_l, self.Jv_bh_l, self.dJvdq_bh_l)

        self.R_wc, self.w_cc, self.p_wc, self.v_wc, self.Jw_cc, self.dJwdq_cc, self.Jv_wc, self.dJvdq_wc = KIN.base2world(self.R_wb, self.w_bb, self.p_wb, self.v_bb, self.R_bc, self.w_bc, self.p_bc, self.v_bc, self.Jw_bc, self.dJwdq_bc, self.Jv_bc, self.dJvdq_bc)

        # dynamics
        self.H, self.CG, self.AG, self.dAGdq, self.p_wg, self.v_wg, self.h = DYN.robot_ID(self.R_wb, self.p_wb, self.w_bb, self.v_bb,
                                                                                          self.q_leg_r, self.q_leg_l, self.q_arm_r, self.q_arm_l, self.q_head,
                                                                                          self.dq_leg_r, self.dq_leg_l, self.dq_arm_r, self.dq_arm_l, self.dq_head)
        
        # foot contacts
        val = np.linalg.pinv(np.hstack((np.vstack((self.Jv_wa_r[:, 6:10].T, np.zeros((3, 4)).T)), np.vstack((np.zeros((3, 4)).T, self.Jv_wa_l[:, 6:10].T))))) @ (np.hstack((self.CG[6:10], self.CG[12:16])) - np.hstack((self.tau_leg_r[0:4], self.tau_leg_l[0:4])))
        for idx in range(3):
            self.foot_contact_force[0, idx] = MF.exp_filter(self.foot_contact_force[0, idx], val[idx],   0.1)
            self.foot_contact_force[1, idx] = MF.exp_filter(self.foot_contact_force[1, idx], val[idx+3], 0.1)
        th = 60 if HARDWARE else 25
        self.foot_contact_state[0] = 1 if self.foot_contact_force[0, 2] > th else 0
        self.foot_contact_state[1] = 1 if self.foot_contact_force[1, 2] > th else 0
        
        # angular momentum about CoP
        if self.robot.locomotion_phase == STANCE_DOUBLE:
            self.p_wcop = 0.5 * (self.p_wa_r + self.p_wa_l)
        elif self.robot.locomotion_phase == STANCE_RIGHT:
            self.p_wcop = np.array(self.p_wa_r)
        elif self.robot.locomotion_phase == STANCE_LEFT:
            self.p_wcop = np.array(self.p_wa_l)

        if self.robot.locomotion_phase in [STANCE_DOUBLE, STANCE_RIGHT, STANCE_LEFT]:
            if HARDWARE:
                bet = 0.1  # must be greater than 0 less than 1
                cop_offset = 0.1
            else:
                bet = 0.1
                cop_offset = 0.1
            vec = self.p_wg - self.p_wcop
            vec[2] += cop_offset
            k_wc_obs = self.h[0:3] + ROBOT_MASS * MF.hat(vec) @ self.v_wg
            self.k_wc[0] = bet * (self.k_wc[0] - ROBOT_MASS * GRAVITY_ACCEL * vec[1] * self.dt) + (1 - bet) * k_wc_obs[0]
            self.k_wc[1] = bet * (self.k_wc[1] + ROBOT_MASS * GRAVITY_ACCEL * vec[0] * self.dt) + (1 - bet) * k_wc_obs[1]

    def set_states(self):
        """
        Save estimated robot states to shared memory
        """
        base_data = {}
        base_data['base_position']        = self.p_wb
        base_data['base_velocity']        = self.v_wb
        base_data['base_acceleration']    = self.a_wb
        base_data['base_rotation_matrix'] = self.R_wb
        base_data['base_heading']         = np.array([self.heading])
        base_data['base_angular_rate']    = self.w_bb
        MM.BASE_STATE.set(base_data)

        com_data = {}
        com_data['com_position']         = self.p_wg
        com_data['com_velocity']         = self.v_wg
        com_data['linear_momentum']      = self.h[3:6]
        com_data['angular_momentum']     = self.h[0:3]
        com_data['angular_momentum_cop'] = self.k_wc
        com_data['H_matrix']             = self.H
        com_data['CG_vector']            = self.CG
        com_data['AG_matrix']            = self.AG
        com_data['dAGdq_vector']         = self.dAGdq
        MM.COM_STATE.set(com_data)

        right_foot_data = {}
        right_foot_data['right_foot_rotation_matrix'] = self.R_wf_r
        right_foot_data['right_foot_angular_rate']    = self.w_ff_r
        right_foot_data['right_foot_Jw']              = self.Jw_ff_r
        right_foot_data['right_foot_dJwdq']           = self.dJwdq_ff_r
        right_foot_data['right_foot_position']        = self.p_wf_r
        right_foot_data['right_foot_velocity']        = self.v_wf_r
        right_foot_data['right_foot_Jv']              = self.Jv_wf_r
        right_foot_data['right_foot_dJvdq']           = self.dJvdq_wf_r
        right_foot_data['right_foot_contact_state']   = self.foot_contact_state[0]
        right_foot_data['right_foot_contact_force']   = self.foot_contact_force[0, :]
        MM.RIGHT_FOOT_STATE.set(right_foot_data)

        left_foot_data = {}
        left_foot_data['left_foot_rotation_matrix'] = self.R_wf_l
        left_foot_data['left_foot_angular_rate']    = self.w_ff_l
        left_foot_data['left_foot_Jw']              = self.Jw_ff_l
        left_foot_data['left_foot_dJwdq']           = self.dJwdq_ff_l
        left_foot_data['left_foot_position']        = self.p_wf_l
        left_foot_data['left_foot_velocity']        = self.v_wf_l
        left_foot_data['left_foot_Jv']              = self.Jv_wf_l
        left_foot_data['left_foot_dJvdq']           = self.dJvdq_wf_l
        left_foot_data['left_foot_contact_state']   = self.foot_contact_state[1]
        left_foot_data['left_foot_contact_force']   = self.foot_contact_force[1, :]
        MM.LEFT_FOOT_STATE.set(left_foot_data)

        right_ankle_data = {}
        right_ankle_data['right_ankle_position'] = self.p_wa_r
        right_ankle_data['right_ankle_velocity'] = self.v_wa_r
        right_ankle_data['right_ankle_Jv']       = self.Jv_wa_r
        right_ankle_data['right_ankle_dJvdq']    = self.dJvdq_wa_r
        MM.RIGHT_ANKLE_STATE.set(right_ankle_data)

        left_ankle_data = {}
        left_ankle_data['left_ankle_position'] = self.p_wa_l
        left_ankle_data['left_ankle_velocity'] = self.v_wa_l
        left_ankle_data['left_ankle_Jv']       = self.Jv_wa_l
        left_ankle_data['left_ankle_dJvdq']    = self.dJvdq_wa_l
        MM.LEFT_ANKLE_STATE.set(left_ankle_data)

        right_hand_data = {}
        right_hand_data['right_hand_rotation_matrix'] = self.R_wh_r
        right_hand_data['right_hand_angular_rate']    = self.w_hh_r
        right_hand_data['right_hand_Jw']              = self.Jw_hh_r
        right_hand_data['right_hand_dJwdq']           = self.dJwdq_hh_r
        right_hand_data['right_hand_position']        = self.p_wh_r
        right_hand_data['right_hand_velocity']        = self.v_wh_r
        right_hand_data['right_hand_Jv']              = self.Jv_wh_r
        right_hand_data['right_hand_dJvdq']           = self.dJvdq_wh_r
        MM.RIGHT_HAND_STATE.set(right_hand_data)

        left_hand_data = {}
        left_hand_data['left_hand_rotation_matrix'] = self.R_wh_l
        left_hand_data['left_hand_angular_rate']    = self.w_hh_l
        left_hand_data['left_hand_Jw']              = self.Jw_hh_l
        left_hand_data['left_hand_dJwdq']           = self.dJwdq_hh_l
        left_hand_data['left_hand_position']        = self.p_wh_l
        left_hand_data['left_hand_velocity']        = self.v_wh_l
        left_hand_data['left_hand_Jv']              = self.Jv_wh_l
        left_hand_data['left_hand_dJvdq']           = self.dJvdq_wh_l
        MM.LEFT_HAND_STATE.set(left_hand_data)

        head_camera_data = {}
        head_camera_data['head_camera_rotation_matrix'] = self.R_wc
        head_camera_data['head_camera_angular_rate']    = self.w_cc
        head_camera_data['head_camera_position']        = self.p_wc
        head_camera_data['head_camera_velocity']        = self.v_wc
        MM.HEAD_CAMERA_STATE.set(head_camera_data)

    def check_sensor(self):
        """
        Check sensor
        """
        imu_count = 0
        imu_data  = []
        imu_error = True
        print("Checking IMU ...")
        import statistics
        while imu_error:
            while imu_count < 500:
                self.robot.update_sense_states()
                imu_data.append(self.robot.imu_angular_rate[2])
                imu_count += 1
                time.sleep(0.002)
            if 0 < np.abs(statistics.mean(imu_data)) < 1:
                imu_error = False
            else:
                imu_count = 0
                print("IMU Not Working. Retrying ...")
        print("IMU OK!")


def main_loop():
    # Frequency
    loop_freq     = 500  # run at 500 Hz
    loop_duration = 1. / loop_freq

    check_thread_frequency = 10
    check_thread_duration  = 1. / check_thread_frequency

    # Themis Setup
    Themis = RDS.RobotDataManager()

    # State Estimation Setup
    se = StateEstimation(robot=Themis, frequency=loop_freq)

    print("====== The State Estimation Thread is running at", loop_freq, "Hz... ======")

    last_check_thread_time = Themis.get_time()

    thread_check = False

    t0 = Themis.get_time()
    while True:
        # time info
        loop_start_time = Themis.get_time()

        # check threading issue
        last_check_thread_elapse = loop_start_time - last_check_thread_time
        if last_check_thread_elapse > check_thread_duration:
            last_check_thread_time = loop_start_time
            if Themis.thread_issue():
                Themis.stop_threading()

        if not thread_check and loop_start_time - t0 > 1:
            if np.all(se.foot_contact_state):
                cprint("THREAD IS DOING GREAT!", 'green')
                MM.THREAD_STATE.set({'estimation': np.array([1])}, opt='update')  # thread is running
                thread_check = True
            else:
                raise RDS.ROBOT_NOT_ON_GROUND

        # run state estimation and update robot states
        se.update()

        # check time to ensure the thread stays at a consistent running loop
        loop_elapsed_time = Themis.get_time() - loop_start_time
        if loop_elapsed_time > loop_duration * 1.5:
            cprint("Delayed " + str(1e3 * (loop_elapsed_time - loop_duration))[0:5] + " ms", 'yellow')
        elif loop_elapsed_time < loop_duration:
            while Themis.get_time() - loop_start_time < loop_duration:
                pass

if __name__ == '__main__':
    if ESTIMATION:
        try:
            main_loop()
        except RDS.ROBOT_NOT_ON_GROUND:
            cprint("ROBOT IS NOT ON THE GROUND!", 'red')
            MM.THREAD_STATE.set({'estimation': np.array([3])}, opt='update')  # robot not on ground
        except RDS.THREAD_STOP:
            cprint("THREAD STOPPED PEACEFULLY!", 'light_grey')
            MM.THREAD_STATE.set({'estimation': np.array([0])}, opt='update')  # thread is stopped
        except (Exception, KeyboardInterrupt) as error:
            print(error)
            cprint("THREAD IN ERROR! TERMINATE NOW!", 'red')
            MM.THREAD_STATE.set({'estimation': np.array([2])}, opt='update')  # thread in error
