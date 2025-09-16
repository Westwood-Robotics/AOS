#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 7, 2024"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
THEMIS Robot Model API
ATTENTION: do not make changes to this file unless you know what you are doing :)
'''

import numpy as np
from Setting.Macros.bear_macros import *
from Library.ROBOT_MODEL import THEMIS_dynamics as DYN
from Library.ROBOT_MODEL import THEMIS_kinematics as KIN


def get_robot_dynamics(R_wb, p_wb, w_bb, v_bb,
                       q_leg_r,  q_leg_l,  q_arm_r,  q_arm_l,  q_head,
                       dq_leg_r, dq_leg_l, dq_arm_r, dq_arm_l, dq_head):
    """
    Compute joint-space equations of motion

    ARGUMENT:
    R_wb (3 x 3) - base rotation matrix in world frame
    p_wb (1 x 3) - base position        in world frame
    w_bb (1 x 3) - base angular rate    in base frame
    v_bb (1 x 3) - base linear velocity in base frame
    q_leg_r  (1 x 6), q_leg_l  (1 x 6), q_arm_r  (1 x 7), q_arm_l  (1 x 7), q_head  (1 x 2) - joint positions
    dq_leg_r (1 x 6), dq_leg_l (1 x 6), dq_arm_r (1 x 7), dq_arm_l (1 x 7), dq_head (1 x 2) - joint velocities

    RETURN:
    H     (34 x 34) - inertia matirx
    CG    ( 1 x 34) - vector of centrifugal, Coriolis, and gravity terms
    AG    ( 6 x 34) - centroidal momentum matrix
    dAGdq ( 1 x  6) - time derivative of AG times dq
    p_wg  ( 1 x  3) - CoM position in world frame
    v_wg  ( 1 x  3) - CoM velocity in world frame
    h     ( 1 x  6) - centroidal momentum, i.e., angular momentum about CoM and linear momentum
    """
    H, CG, AG, dAGdq, p_wg, v_wg, h = DYN.robot_ID(R_wb, p_wb, w_bb, v_bb,
                                                   q_leg_r, q_leg_l, q_arm_r, q_arm_l, q_head,
                                                   dq_leg_r, dq_leg_l, dq_arm_r, dq_arm_l, dq_head)
    return H, CG, AG, dAGdq, p_wg, v_wg, h


def get_leg_transformations(leg, location, T_wb, q):
    """
    Compute transformation matrices for each frame on leg in world frame

    ARGUMENT:
    leg      (  int) - leg index, i.e., right leg (+1), left leg (-1)
    location (  int) - end frame location index, i.e., ankle (0), foot middle (1), inner toe (2), outer toe (3), heel (4)
    T_wb     (4 x 4) - transformation matrix of base in world frame
    q        (1 x 6) - leg joint positions

    RETURN:
    T (6 x 4 x 4) - transformation matrices of each frame on leg in world frame
    """
    T = KIN.chain_TF(leg, location, T_wb, q)
    return T


def get_arm_transformations(arm, location, T_wb, q):
    """
    Compute transformation matrices for each frame on arm in world frame

    ARGUMENT:
    arm      (  int) - arm index, i.e., right arm (+1), left arm (-1)
    location (  int) - end frame location index, i.e., wrist (0), hand (1)
    T_wb     (4 x 4) - transformation matrix of base in world frame
    q        (1 x 7) - arm joint positions

    RETURN:
    T (7 x 4 x 4) - transformation matrices of each frame on arm in world frame
    """
    T = KIN.chain_TF(arm * 2, location, T_wb, q)
    return T


def get_head_transformations(head, location, T_wb, q):
    """
    Compute transformation matrices for each frame on head in world frame

    ARGUMENT:
    head     (  int) - head index (0)
    location (  int) - end frame location index, i.e., neck (0), camera (1)
    T_wb     (4 x 4) - transformation matrix of base in world frame
    q        (1 x 2) - head joint positions

    RETURN:
    T (2 x 4 x 4) - transformation matrices of each frame on head in world frame
    """
    T = KIN.chain_TF(head, location, T_wb, q)
    return T


def get_leg_pose(leg, location, T, q0):
    """
    Compute leg joint positions given foot transformation matrix in base frame (check "solve_foot_IK" function in robot_data.py for how to use)

    ARGUMENT:
    leg      (  int) - leg index, i.e., right leg (+1), left leg (-1)
    location (  int) - end frame location index, i.e., ankle (0), foot middle (1), inner toe (2), outer toe (3), heel (4)
    T        (4 x 4) - transformation matrix of foot frame in base frame
    q0       (1 x 6) - initial guess

    RETURN:
    q (4 x 6) - all possible solutions with a maximum number of 4
    """
    q = KIN.foot_IK(leg, location, T[0:3, 0:3], T[0:3, 3], q0)
    return q


def get_arm_pose(arm, location, T, q0):
    """
    Compute arm joint positions given hand transformation matrix in base frame (check "solve_hand_IK" function in robot_data.py for how to use)

    ARGUMENT:
    arm      (  int) - arm index, i.e., right arm (+1), left arm (-1)
    location (  int) - end frame location index, i.e., wrist (0), hand (1)
    T        (4 x 4) - transformation matrix of foot frame in base frame
    q0       (1 x 7) - initial guess

    RETURN:
    q (1 x 7) - all possible solutions with a maximum number of 1
    """
    q = KIN.hand_IK(arm, location, T[0:3, 0:3], T[0:3, 3], q0)
    return q


def get_foot_FK(leg, location, q, dq):
    """
    Compute transformation matrix and Jacobian for foot frame in base frame

    ARGUMENT:
    leg      (  int) - leg index, i.e., right leg (+1), left leg (-1)
    location (  int) - end frame location index, i.e., ankle (0), foot middle (1), inner toe (2), outer toe (3), heel (4)
    q        (1 x 6) - leg joint positions
    dq       (1 x 6) - leg joint velocities

    RETURN:
    T    (4 x 4) - transformation matrix of foot frame in base frame
    v    (1 x 6) - angular and linear velocity of foot frame in base frame
    J    (6 x 6) - angular and linear Jacobian of foot frame in base frame
    dJdq (1 x 6) - time derivative of J times dq
    """
    R, w, p, v, Jw, dJwdq, Jv, dJvdq = KIN.chain_FK(leg, location, q, dq)

    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3,   3] = p

    v = np.zeros(6)
    v[0:3] = w
    v[3:6] = v

    J = np.zeros((6, 6))
    J[0:3, 0:6] = Jw
    J[3:6, 0:6] = Jv

    dJdq = np.zeros(6)
    dJdq[0:3] = dJwdq
    dJdq[3:6] = dJvdq

    return T, v, J, dJdq


def get_hand_FK(arm, location, q, dq):
    """
    Compute transformation matrix and Jacobian for hand frame in base frame

    ARGUMENT:
    arm      (  int) - leg index, i.e., right arm (+1), left arm (-1)
    location (  int) - end frame location index, i.e., wrist (0), hand (1)
    q        (1 x 7) - arm joint positions
    dq       (1 x 7) - arm joint velocities

    RETURN:
    T    (4 x 4) - transformation matrix of hand frame in base frame
    v    (1 x 6) - angular and linear velocity of hand frame in base frame
    J    (6 x 7) - angular and linear Jacobian of hand frame in base frame
    dJdq (1 x 6) - time derivative of J times dq
    """
    R, w, p, v, Jw, dJwdq, Jv, dJvdq = KIN.chain_FK(arm * 2, location, q, dq)

    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3,   3] = p

    v = np.zeros(6)
    v[0:3] = w
    v[3:6] = v

    J = np.zeros((6, 7))
    J[0:3, 0:7] = Jw
    J[3:6, 0:7] = Jv

    dJdq = np.zeros(6)
    dJdq[0:3] = dJwdq
    dJdq[3:6] = dJvdq

    return T, v, J, dJdq


def get_joint_states(sim, chain, option, m):
    """
    Convert motor states to joint states

    ARGUMENT:
    sim    (  int) - hardware or simulation, i.e., hardware (0), simulation (1)
    chain  (  int) - limb chain index, i.e., head (0), right leg (+1), left leg (-1), right arm (+2), left arm (-2), right hand (+3), left hand (-3)
    option (  int) - state property, i.e., position (0), velocity (1), torque (2)
    m      (1 x n) - motor states

    RETURN:
    q      (1 x n) - joint states
    """
    q = KIN.motor2joint(sim, chain, option, m)
    return q


def get_motor_states(sim, chain, option, q):
    """
    Convert joint states to motor states

    ARGUMENT:
    sim    (  int) - hardware or simulation, i.e., hardware (0), simulation (1)
    chain  (  int) - limb chain index, i.e., head (0), right leg (+1), left leg (-1), right arm (+2), left arm (-2), right hand (+3), left hand (-3)
    option (  int) - state property, i.e., position (0), velocity (1), torque (2)
    q      (1 x n) - joint states

    RETURN:
    m      (1 x n) - motor states
    """
    m = KIN.joint2motor(sim, chain, option, q)
    return m


# motor2joint
T_bj_leg_r = np.array([[+1.,  0.,  0.,  0.,   0.,   0.],
                       [ 0., +1.,  0.,  0.,   0.,   0.],
                       [ 0.,  0., +1.,  0.,   0.,   0.],
                       [ 0.,  0.,  0., -1.,   0.,   0.],
                       [ 0.,  0.,  0.,  0., -0.5, +0.5],
                       [ 0.,  0.,  0.,  0., -1.0, -1.0]])

T_bj_leg_l = np.array([[+1.,  0.,  0.,  0.,   0.,   0.],
                       [ 0., +1.,  0.,  0.,   0.,   0.],
                       [ 0.,  0., -1.,  0.,   0.,   0.],
                       [ 0.,  0.,  0., +1.,   0.,   0.],
                       [ 0.,  0.,  0.,  0., +0.5, -0.5],
                       [ 0.,  0.,  0.,  0., -1.0, -1.0]])

T_bj_arm_r = np.array([[+1.,  0.,  0.,  0.,  0.,  0.,  0.],
                       [ 0., +1.,  0.,  0.,  0.,  0.,  0.],
                       [ 0.,  0., +1.,  0.,  0.,  0.,  0.],
                       [ 0.,  0.,  0., -1.,  0.,  0.,  0.],
                       [ 0.,  0.,  0.,  0., +1.,  0.,  0.],
                       [ 0.,  0.,  0.,  0.,  0., -1.,  0.],
                       [ 0.,  0.,  0.,  0.,  0.,  0., +1.]])

T_bj_arm_l = np.array([[-1.,  0.,  0.,  0.,  0.,  0.,  0.],
                       [ 0., +1.,  0.,  0.,  0.,  0.,  0.],
                       [ 0.,  0., +1.,  0.,  0.,  0.,  0.],
                       [ 0.,  0.,  0., -1.,  0.,  0.,  0.],
                       [ 0.,  0.,  0.,  0., +1.,  0.,  0.],
                       [ 0.,  0.,  0.,  0.,  0., -1.,  0.],
                       [ 0.,  0.,  0.,  0.,  0.,  0., +1.]])

T_bj_head = np.array([[+1.,  0.],
                      [ 0., -1.]])

T_dj_hand_r = np.diag([-1., -1., 1., 1., 1., 1., 1.])

T_dj_hand_l = np.diag([-1., -1., 1., 1., 1., 1., 1.])

# joint2motor
T_jb_leg_r  = np.linalg.inv(T_bj_leg_r)
T_jb_leg_l  = np.linalg.inv(T_bj_leg_l)
T_jb_arm_r  = np.linalg.inv(T_bj_arm_r)
T_jb_arm_l  = np.linalg.inv(T_bj_arm_l)
T_jb_head   = np.linalg.inv(T_bj_head)
T_jd_hand_r = np.linalg.inv(T_dj_hand_r)
T_jd_hand_l = np.linalg.inv(T_dj_hand_l)

# iq2torque
T_it_leg_r = np.diag([BEAR[BEAR_TORQUE_CONSTANT][BEAR_HIP_YAW_R],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_HIP_ROLL_R],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_HIP_PITCH_R],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_KNEE_R],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_ANKLE_IN_R],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_ANKLE_OUT_R]])

T_it_leg_l = np.diag([BEAR[BEAR_TORQUE_CONSTANT][BEAR_HIP_YAW_L],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_HIP_ROLL_L],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_HIP_PITCH_L],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_KNEE_L],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_ANKLE_IN_L],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_ANKLE_OUT_L]])

T_it_arm_r = np.diag([BEAR[BEAR_TORQUE_CONSTANT][BEAR_SHOULDER_PITCH_R],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_SHOULDER_ROLL_R],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_SHOULDER_YAW_R],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_ELBOW_PITCH_R],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_ELBOW_YAW_R],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_WRIST_PITCH_R],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_WRIST_YAW_R]])

T_it_arm_l = np.diag([BEAR[BEAR_TORQUE_CONSTANT][BEAR_SHOULDER_PITCH_L],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_SHOULDER_ROLL_L],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_SHOULDER_YAW_L],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_ELBOW_PITCH_L],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_ELBOW_YAW_L],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_WRIST_PITCH_L],
                      BEAR[BEAR_TORQUE_CONSTANT][BEAR_WRIST_YAW_L]])

T_it_head = np.diag([BEAR[BEAR_TORQUE_CONSTANT][BEAR_HEAD_YAW],
                     BEAR[BEAR_TORQUE_CONSTANT][BEAR_HEAD_PITCH]])

T_it_hand_r = np.diag([-1., -1., 1., 1., 1., 1., 1.])
T_it_hand_l = np.diag([-1., -1., 1., 1., 1., 1., 1.])

# torque2iq
T_ti_leg_r  = np.linalg.inv(T_it_leg_r)
T_ti_leg_l  = np.linalg.inv(T_it_leg_l)
T_ti_arm_r  = np.linalg.inv(T_it_arm_r)
T_ti_arm_l  = np.linalg.inv(T_it_arm_l)
T_ti_head   = np.linalg.inv(T_it_head)
T_ti_hand_r = np.linalg.inv(T_it_hand_r)
T_ti_hand_l = np.linalg.inv(T_it_hand_l)