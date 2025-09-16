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

import numpy as np
import Setting.robot_data as RDS
from Play.config import *
from Setting.Macros.model_macros import *
from Test.Actuator.set_joint import move_to_goal_joint_positions


if __name__ == '__main__':
    Themis = RDS.RobotDataManager()

    # move fingers to collision-free position
    Themis.update_joint_states(chains=ROBOT_CHAIN_LIST)
    q_hand_r = np.array([1.57, 0.00, 1.57, 0.00, -0.50, 2.00, Themis.joint_position[JOINT_PALM_AXIAL_R]])
    q_hand_l = np.array([1.57, 0.00, 1.57, 0.00, -0.50, 2.00, Themis.joint_position[JOINT_PALM_AXIAL_L]])

    Ns = 100
    Td = 0.01
    move_to_goal_joint_positions(Themis, Ns, Td,
                                 left_hand_goal_positions=q_hand_l,
                                 right_hand_goal_positions=q_hand_r)

    # move to initial position
    Themis.update_joint_states(chains=ROBOT_CHAIN_LIST)
    q0_leg_r = np.array([Themis.joint_position[key] for key in LEG_R_JOINT_ID_LIST])
    q0_leg_l = np.array([Themis.joint_position[key] for key in LEG_L_JOINT_ID_LIST])

    # arm pose
    # q_arm_r = np.array([-0.10, +1.40, +1.57, +0.40, 0.00, 0.00])
    # q_arm_l = np.array([-0.10, -1.40, -1.57, -0.40, 0.00, 0.00])
    q_arm_r = np.array([-0.20, +1.40, +1.57, +0.40, -0.00, -0.00, -1.50])
    q_arm_l = np.array([-0.20, -1.40, -1.57, -0.40, +0.00, +0.00, +1.50])
    

    # hand pose
    # q_hand_r = np.array([0.50, 2.00, 0.50, 2.00, 0.50, 2.00, 1.50])
    # q_hand_l = np.array([0.50, 2.00, 0.50, 2.00, 0.50, 2.00, 1.50])
    q_hand_r = np.array([2.00, 1.90, 2.00, 1.90, 2.00, 1.80, 3.00])
    q_hand_l = np.array([2.00, 1.90, 2.00, 1.90, 2.00, 1.80, 3.00])

    # head pose
    q_head = np.array([0., 0.])
    
    # leg pose 
    p_ba_r = np.array([-0.11, -0.11, -1.05]) if HARDWARE else np.array([-0.15, -0.11, -1.05])
    p_ba_l = np.array([-0.11, +0.11, -1.05]) if HARDWARE else np.array([-0.15, +0.11, -1.05])

    R_ba_r = np.eye(3)
    R_ba_l = np.eye(3)

    q_leg_r = Themis.solve_foot_IK(leg='right', location='ankle', R=R_ba_r, p=p_ba_r, q0=q0_leg_r)
    q_leg_l = Themis.solve_foot_IK(leg='left',  location='ankle', R=R_ba_l, p=p_ba_l, q0=q0_leg_l)

    print(q_leg_l, q_leg_r)

    # print(q_leg_r)
    Ns = 200
    Td = 0.01
    move_to_goal_joint_positions(Themis, Ns, Td,
                                 right_leg_goal_positions=q_leg_r,
                                 left_leg_goal_positions=q_leg_l,
                                 right_arm_goal_positions=q_arm_r,
                                 left_arm_goal_positions=q_arm_l,
                                 head_goal_positions=q_head,
                                 right_hand_goal_positions=q_hand_r,
                                 left_hand_goal_positions=q_hand_l)
