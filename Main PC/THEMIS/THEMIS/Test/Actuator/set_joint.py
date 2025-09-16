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
Set joint positions
'''

import time
import numpy as np
import Setting.robot_data as RDS
from collections import defaultdict
from Setting.Macros.bear_macros import *
from Setting.Macros.model_macros import *


def move_to_goal_joint_positions(robot, npt, dt,
                                 right_leg_goal_positions=None,
                                 left_leg_goal_positions=None,
                                 right_arm_goal_positions=None,
                                 left_arm_goal_positions=None,
                                 head_goal_positions=None,
                                 right_hand_goal_positions=None,
                                 left_hand_goal_positions=None,
                                 right_leg_initial_positions=None,
                                 left_leg_initial_positions=None,
                                 right_arm_initial_positions=None,
                                 left_arm_initial_positions=None,
                                 head_initial_positions=None,
                                 right_hand_initial_positions=None,
                                 left_hand_initial_positions=None):
    
    joint_id_list = []
    chain_id_list = []
    traj_list = defaultdict(int)

    if right_leg_goal_positions is not None:
        joint_id_list += LEG_R_JOINT_ID_LIST
        chain_id_list += [CHAIN_LEG_R]
        if right_leg_initial_positions is not None:
            for idx, joint_id in enumerate(LEG_R_JOINT_ID_LIST):
                traj_list[joint_id] = np.linspace(right_leg_initial_positions[idx], right_leg_goal_positions[idx], npt, endpoint=True)
        else:
            robot.update_joint_states(chains=[CHAIN_LEG_R])
            for idx, joint_id in enumerate(LEG_R_JOINT_ID_LIST):
                traj_list[joint_id] = np.linspace(robot.joint_position[joint_id], right_leg_goal_positions[idx], npt, endpoint=True)

    if left_leg_goal_positions is not None:
        joint_id_list += LEG_L_JOINT_ID_LIST
        chain_id_list += [CHAIN_LEG_L]
        robot.update_joint_states(chains=[CHAIN_LEG_L])
        if left_leg_initial_positions is not None:
            for idx, joint_id in enumerate(LEG_L_JOINT_ID_LIST):
                traj_list[joint_id] = np.linspace(left_leg_initial_positions[idx], left_leg_goal_positions[idx], npt, endpoint=True)
        else:
            for idx, joint_id in enumerate(LEG_L_JOINT_ID_LIST):
                traj_list[joint_id] = np.linspace(robot.joint_position[joint_id], left_leg_goal_positions[idx], npt, endpoint=True)

    if right_arm_goal_positions is not None:
        joint_id_list += ARM_R_JOINT_ID_LIST
        chain_id_list += [CHAIN_ARM_R]
        robot.update_joint_states(chains=[CHAIN_ARM_R])
        if right_arm_initial_positions is not None:
            for idx, joint_id in enumerate(ARM_R_JOINT_ID_LIST):
                traj_list[joint_id] = np.linspace(right_arm_initial_positions[idx], right_arm_goal_positions[idx], npt, endpoint=True)
        else:
            for idx, joint_id in enumerate(ARM_R_JOINT_ID_LIST):
                traj_list[joint_id] = np.linspace(robot.joint_position[joint_id], right_arm_goal_positions[idx], npt, endpoint=True)

    if left_arm_goal_positions is not None:
        joint_id_list += ARM_L_JOINT_ID_LIST
        chain_id_list += [CHAIN_ARM_L]
        robot.update_joint_states(chains=[CHAIN_ARM_L])
        if left_arm_initial_positions is not None:
            for idx, joint_id in enumerate(ARM_L_JOINT_ID_LIST):
                traj_list[joint_id] = np.linspace(left_arm_initial_positions[idx], left_arm_goal_positions[idx], npt, endpoint=True)
        else:
            for idx, joint_id in enumerate(ARM_L_JOINT_ID_LIST):
                traj_list[joint_id] = np.linspace(robot.joint_position[joint_id], left_arm_goal_positions[idx], npt, endpoint=True)

    if head_goal_positions is not None:
        joint_id_list += HEAD_JOINT_ID_LIST
        chain_id_list += [CHAIN_HEAD]
        robot.update_joint_states(chains=[CHAIN_HEAD])
        if head_initial_positions is not None:
            for idx, joint_id in enumerate(HEAD_JOINT_ID_LIST):
                traj_list[joint_id] = np.linspace(head_initial_positions[idx], head_goal_positions[idx], npt, endpoint=True)
        else:
            for idx, joint_id in enumerate(HEAD_JOINT_ID_LIST):
                traj_list[joint_id] = np.linspace(robot.joint_position[joint_id], head_goal_positions[idx], npt, endpoint=True)

    if right_hand_goal_positions is not None:
        joint_id_list += HAND_R_JOINT_ID_LIST
        chain_id_list += [CHAIN_HAND_R]
        robot.update_joint_states(chains=[CHAIN_HAND_R])
        if right_hand_initial_positions is not None:
            for idx, joint_id in enumerate(HAND_R_JOINT_ID_LIST):
                traj_list[joint_id] = np.linspace(right_hand_initial_positions[idx], right_hand_goal_positions[idx], npt, endpoint=True)
        else:
            for idx, joint_id in enumerate(HAND_R_JOINT_ID_LIST):
                traj_list[joint_id] = np.linspace(robot.joint_position[joint_id], right_hand_goal_positions[idx], npt, endpoint=True)

    if left_hand_goal_positions is not None:
        joint_id_list += HAND_L_JOINT_ID_LIST
        chain_id_list += [CHAIN_HAND_L]
        robot.update_joint_states(chains=[CHAIN_HAND_L])
        if left_hand_initial_positions is not None:
            for idx, joint_id in enumerate(HAND_L_JOINT_ID_LIST):
                traj_list[joint_id] = np.linspace(left_hand_initial_positions[idx], left_hand_goal_positions[idx], npt, endpoint=True)
        else:
            for idx, joint_id in enumerate(HAND_L_JOINT_ID_LIST):
                traj_list[joint_id] = np.linspace(robot.joint_position[joint_id], left_hand_goal_positions[idx], npt, endpoint=True)

    for tdx in range(npt):
        for joint_id in joint_id_list:
            robot.goal_joint_position[joint_id] = traj_list[joint_id][tdx]
        robot.set_joint_positions(chains=chain_id_list)
        time.sleep(dt)
        # robot.delay_time(dt)


if __name__ == '__main__':
    Themis = RDS.RobotDataManager()

    Themis.update_joint_states(chains=ROBOT_CHAIN_LIST)
    lr1 = Themis.joint_position[JOINT_HIP_YAW_R]
    lr2 = Themis.joint_position[JOINT_HIP_ROLL_R]
    lr3 = Themis.joint_position[JOINT_HIP_PITCH_R]
    lr4 = Themis.joint_position[JOINT_KNEE_PITCH_R]
    lr5 = Themis.joint_position[JOINT_ANKLE_PITCH_R]
    lr6 = Themis.joint_position[JOINT_ANKLE_ROLL_R]

    ll1 = Themis.joint_position[JOINT_HIP_YAW_L]
    ll2 = Themis.joint_position[JOINT_HIP_ROLL_L]
    ll3 = Themis.joint_position[JOINT_HIP_PITCH_L]
    ll4 = Themis.joint_position[JOINT_KNEE_PITCH_L]
    ll5 = Themis.joint_position[JOINT_ANKLE_PITCH_L]
    ll6 = Themis.joint_position[JOINT_ANKLE_ROLL_L]

    lh1 = Themis.joint_position[JOINT_INDEX_MCP_PITCH_L]
    lh2 = Themis.joint_position[JOINT_INDEX_PIP_PITCH_L]
    lh3 = Themis.joint_position[JOINT_MIDDLE_MCP_PITCH_L]
    lh4 = Themis.joint_position[JOINT_MIDDLE_PIP_PITCH_L]
    lh5 = Themis.joint_position[JOINT_THUMB_MCP_PITCH_L]
    lh6 = Themis.joint_position[JOINT_THUMB_PIP_PITCH_L]
    lh7 = Themis.joint_position[JOINT_PALM_AXIAL_L]

    rh1 = Themis.joint_position[JOINT_INDEX_MCP_PITCH_R]
    rh2 = Themis.joint_position[JOINT_INDEX_PIP_PITCH_R]
    rh3 = Themis.joint_position[JOINT_MIDDLE_MCP_PITCH_R]
    rh4 = Themis.joint_position[JOINT_MIDDLE_PIP_PITCH_R]
    rh5 = Themis.joint_position[JOINT_THUMB_MCP_PITCH_R]
    rh6 = Themis.joint_position[JOINT_THUMB_PIP_PITCH_R]
    rh7 = Themis.joint_position[JOINT_PALM_AXIAL_R]

    Ns = 100   # number of pieces
    Td = 0.01  # time delay [sec] between adjacent pieces

    # Start moving
    confirm = input("Go to goal pose? (y/n) ")
    if confirm != 'y':
        exit()
    
    move_to_goal_joint_positions(Themis, Ns, Td,
                                #  right_leg_goal_positions = np.array([lr1, lr2, lr3, lr4, lr5, lr6]),
                                #  left_leg_goal_positions  = np.array([ll1, ll2, ll3, ll4, ll5, ll6]),
                                 right_hand_goal_positions= np.array([-0.5, +0.5, -0.5, +0.5, -0.5, +0.5, -0.5]),
                                 left_hand_goal_positions = np.array([-0.5, +0.5, -0.5, +0.5, -0.5, +0.5, -0.5]))

    time.sleep(1)

    Themis.disable_dxls(chains=HAND_CHAIN_LIST)