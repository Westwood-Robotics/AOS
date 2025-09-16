#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 18, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Script for setting initial pose of THEMIS
'''

import time
import numpy as np
import Setting.robot_data as RDS
from Play.config import *
from Setting.Macros.model_macros import *
from Test.Actuator.set_joint import move_to_goal_joint_positions


if __name__ == '__main__':
    Themis = RDS.RobotDataManager()

    # move fingers to collision-free position
    Themis.update_joint_states(chains=ROBOT_CHAIN_LIST)
    q_hand_r0 = np.array([1.57, 0.00, 1.57, 0.00, -0.50, 2.00, Themis.joint_position[JOINT_PALM_AXIAL_R]])
    q_hand_l0 = np.array([1.57, 0.00, 1.57, 0.00, -0.50, 2.00, Themis.joint_position[JOINT_PALM_AXIAL_L]])

    Ns = 100
    Td = 0.01
    move_to_goal_joint_positions(Themis, Ns, Td,
                                 left_hand_goal_positions=q_hand_l0,
                                 right_hand_goal_positions=q_hand_r0)
    
    # move to initial position
    # hand pose
    q_hand_r1 = np.array([2.00, 1.90, 2.00, 1.90, 2.00, 1.80, 3.00])
    q_hand_l1 = np.array([2.00, 1.90, 2.00, 1.90, 2.00, 1.80, 3.00])
    move_to_goal_joint_positions(Themis, Ns, Td,
                                 left_hand_goal_positions=q_hand_l1,
                                 right_hand_goal_positions=q_hand_r1,
                                 left_hand_initial_positions=q_hand_l0,
                                 right_hand_initial_positions=q_hand_r0)

    # key frames
    steps = 4
    QL = np.zeros((steps, 28))

    if SIMULATION:
        # 
        QL[0, :] = np.array([ 0.00,  0.00, +1.30, -1.85, +0.55,  0.00,
                              0.00,  0.00, +1.30, -1.85, +0.55,  0.00,
                             -0.20, +1.40, +1.57, +0.40, -0.00, -0.00, -1.50,
                             -0.20, -1.40, -1.57, -0.40,  0.00,  0.00, +1.50,
                              0.00,  0.00])
        QL[1, :] = np.array([ 0.00,  0.00, +1.30, -1.85, +0.55,  0.00,
                              0.00,  0.00, +1.30, -1.85, +0.55,  0.00,
                             -0.20, +1.40, +1.57, +0.40,  0.00,  0.00, -1.50,
                             -0.20, -1.40, -1.57, -0.40,  0.00,  0.00, +1.50,
                              0.00,  0.00])
        QL[2, :] = np.array([ 0.00,  0.00, +1.75, -1.75, +0.60,  0.00,
                              0.00,  0.00, +1.75, -1.75, +0.60,  0.00,
                             -0.20, +1.40, +1.57, +0.40,  0.00,  0.00, -1.50,
                             -0.20, -1.40, -1.57, -0.40,  0.00,  0.00, +1.50,
                              0.00,  0.00])
        QL[3, :] = np.array([ 0.00,  0.00, +0.45, -1.00, +0.55,  0.00,
                              0.00,  0.00, +0.45, -1.00, +0.55,  0.00,
                             -0.20, +1.40, +1.57, +0.40,  0.00,  0.00, -1.50,
                             -0.20, -1.40, -1.57, -0.40,  0.00,  0.00, +1.50,
                              0.00,  0.00])
        Ns = [ 100,  100,  200,  200]
        Td = [0.01, 0.01, 0.01, 0.01]
    else:
        QL[0, :] = np.array([-0.05,  0.00, +1.30, -1.85, +0.55,  0.00,
                             +0.05,  0.00, +1.30, -1.85, +0.55,  0.00,
                             -0.20, +1.40, +1.57, +0.40,  0.00,  0.00, -1.50,
                             -0.20, -1.40, -1.57, -0.40,  0.00,  0.00, +1.50,
                              0.00,  0.00])
        QL[1, :] = np.array([-0.05,  0.00, +1.30, -1.85, +0.55,  0.00,
                             +0.05,  0.00, +1.30, -1.85, +0.55,  0.00,
                             -0.20, +1.40, +1.57, +0.40,  0.00,  0.00, -1.50,
                             -0.20, -1.40, -1.57, -0.40,  0.00,  0.00, +1.50,
                              0.00,  0.00])
        QL[2, :] = np.array([-0.05,  0.00, +1.75, -1.75, +0.60,  0.00,
                             +0.05,  0.00, +1.75, -1.75, +0.60,  0.00,
                             -0.20, +1.40, +1.57, +0.40,  0.00,  0.00, -1.50,
                             -0.20, -1.40, -1.57, -0.40,  0.00,  0.00, +1.50,
                              0.00,  0.00])
        QL[3, :] = np.array([-0.05,  0.00, +0.40, -0.96, +0.51,  0.00,
                             +0.05,  0.00, +0.40, -0.96, +0.51,  0.00,
                             -0.20, +1.40, +1.57, +0.40,  0.00,  0.00, -1.50,
                             -0.20, -1.40, -1.57, -0.40,  0.00,  0.00, +1.50,
                              0.00,  0.00])
        Ns = [ 200,  200,  300,  300]
        Td = [0.01, 0.01, 0.01, 0.01]


    
    # position control to stand from chair
    for idx in range(4):
        q_leg_r1 = QL[idx, 0:6]
        q_leg_l1 = QL[idx, 6:12]
        q_arm_r1 = QL[idx, 12:19]
        q_arm_l1 = QL[idx, 19:26]
        q_head1  = QL[idx, 26:28]
        q_leg_r0 = QL[idx-1, 0:6]   if idx > 0 else None
        q_leg_l0 = QL[idx-1, 6:12]  if idx > 0 else None
        q_arm_r0 = QL[idx-1, 12:19] if idx > 0 else None
        q_arm_l0 = QL[idx-1, 19:26] if idx > 0 else None
        q_head0  = QL[idx-1, 26:28] if idx > 0 else None
        move_to_goal_joint_positions(Themis, Ns[idx], Td[idx],
                                     right_leg_goal_positions=q_leg_r1,
                                     left_leg_goal_positions=q_leg_l1,
                                     right_arm_goal_positions=q_arm_r1,
                                     left_arm_goal_positions=q_arm_l1,
                                     head_goal_positions=q_head1,

                                     right_leg_initial_positions=q_leg_r0,
                                     left_leg_initial_positions=q_leg_l0,
                                     right_arm_initial_positions=q_arm_r0,
                                     left_arm_initial_positions=q_arm_l0,
                                     head_initial_positions=q_head0)
        # time.sleep(1)
