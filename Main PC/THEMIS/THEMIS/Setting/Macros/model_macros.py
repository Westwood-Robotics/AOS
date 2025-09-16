#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 3, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Script that holds useful model macros
'''

import numpy as np
import Library.MATH_FUNCTION.math_function as MF
from collections import defaultdict
from Setting.Macros.constant_macros import *


# ==================================================
# ==================================================
# CHAIN INFO
# ==================================================
# ==================================================

# ------------------------------
# CHAIN PROPERTY
# ------------------------------
CHAIN_LEG_R = 0
CHAIN_LEG_L = 1
CHAIN_ARM_R = 2
CHAIN_ARM_L = 3
CHAIN_HEAD  = 4

CHAIN_HAND_R = 5
CHAIN_HAND_L = 6

CHAIN_HAND = 9
CHAIN_ARM = 10
CHAIN_UPPER_BODY = 11

LEG_CHAIN_LIST = [CHAIN_LEG_R, CHAIN_LEG_L]
ARM_CHAIN_LIST = [CHAIN_ARM_R, CHAIN_ARM_L]
HAND_CHAIN_LIST = [CHAIN_HAND_R, CHAIN_HAND_L]

UPPER_BODY_CHAIN_LIST = [CHAIN_ARM_R, CHAIN_ARM_L, CHAIN_HEAD, CHAIN_HAND_R, CHAIN_HAND_L]
LOWER_BODY_CHAIN_LIST = [CHAIN_LEG_R, CHAIN_LEG_L]
ROBOT_CHAIN_LIST      = LOWER_BODY_CHAIN_LIST + UPPER_BODY_CHAIN_LIST


# ==================================================
# ==================================================
# JOINT INFO
# ==================================================
# ==================================================
JOINT = defaultdict(lambda: defaultdict(int))

# ------------------------------
# JOINT PROPERTY
# ------------------------------
JOINT_ID             = 0
JOINT_POSITION_LIMIT = 1

JOINT_BASE             = 0

JOINT_HIP_YAW_R        = 1
JOINT_HIP_ROLL_R       = 2
JOINT_HIP_PITCH_R      = 3
JOINT_KNEE_PITCH_R     = 4
JOINT_ANKLE_PITCH_R    = 5
JOINT_ANKLE_ROLL_R     = 6

JOINT_HIP_YAW_L        = 7
JOINT_HIP_ROLL_L       = 8
JOINT_HIP_PITCH_L      = 9
JOINT_KNEE_PITCH_L     = 10
JOINT_ANKLE_PITCH_L    = 11
JOINT_ANKLE_ROLL_L     = 12

JOINT_SHOULDER_PITCH_R = 13
JOINT_SHOULDER_ROLL_R  = 14
JOINT_SHOULDER_YAW_R   = 15
JOINT_ELBOW_PITCH_R    = 16
JOINT_ELBOW_YAW_R      = 17
JOINT_WRIST_PITCH_R    = 18
JOINT_WRIST_YAW_R      = 19

JOINT_SHOULDER_PITCH_L = 20
JOINT_SHOULDER_ROLL_L  = 21
JOINT_SHOULDER_YAW_L   = 22
JOINT_ELBOW_PITCH_L    = 23
JOINT_ELBOW_YAW_L      = 24
JOINT_WRIST_PITCH_L    = 25
JOINT_WRIST_YAW_L      = 26

JOINT_HEAD_YAW         = 27
JOINT_HEAD_PITCH       = 28

JOINT_INDEX_MCP_PITCH_R  = 29
JOINT_INDEX_PIP_PITCH_R  = 30
JOINT_MIDDLE_MCP_PITCH_R = 31
JOINT_MIDDLE_PIP_PITCH_R = 32
JOINT_THUMB_MCP_PITCH_R  = 33
JOINT_THUMB_PIP_PITCH_R  = 34
JOINT_PALM_AXIAL_R       = 35

JOINT_INDEX_MCP_PITCH_L  = 36
JOINT_INDEX_PIP_PITCH_L  = 37
JOINT_MIDDLE_MCP_PITCH_L = 38
JOINT_MIDDLE_PIP_PITCH_L = 39
JOINT_THUMB_MCP_PITCH_L  = 40
JOINT_THUMB_PIP_PITCH_L  = 41
JOINT_PALM_AXIAL_L       = 42

# ------------------------------
# JOINT ID
# ------------------------------
BASE_JOINT_ID_LIST  = [JOINT_BASE]      
LEG_R_JOINT_ID_LIST = [JOINT_HIP_YAW_R, JOINT_HIP_ROLL_R, JOINT_HIP_PITCH_R, JOINT_KNEE_PITCH_R, JOINT_ANKLE_PITCH_R, JOINT_ANKLE_ROLL_R]
LEG_L_JOINT_ID_LIST = [JOINT_HIP_YAW_L, JOINT_HIP_ROLL_L, JOINT_HIP_PITCH_L, JOINT_KNEE_PITCH_L, JOINT_ANKLE_PITCH_L, JOINT_ANKLE_ROLL_L]
ARM_R_JOINT_ID_LIST = [JOINT_SHOULDER_PITCH_R, JOINT_SHOULDER_ROLL_R, JOINT_SHOULDER_YAW_R, JOINT_ELBOW_PITCH_R, JOINT_ELBOW_YAW_R, JOINT_WRIST_PITCH_R, JOINT_WRIST_YAW_R]
ARM_L_JOINT_ID_LIST = [JOINT_SHOULDER_PITCH_L, JOINT_SHOULDER_ROLL_L, JOINT_SHOULDER_YAW_L, JOINT_ELBOW_PITCH_L, JOINT_ELBOW_YAW_L, JOINT_WRIST_PITCH_L, JOINT_WRIST_YAW_L]
HEAD_JOINT_ID_LIST  = [JOINT_HEAD_YAW, JOINT_HEAD_PITCH]

HAND_R_JOINT_ID_LIST = [JOINT_INDEX_MCP_PITCH_R, JOINT_INDEX_PIP_PITCH_R, JOINT_MIDDLE_MCP_PITCH_R, JOINT_MIDDLE_PIP_PITCH_R, JOINT_THUMB_MCP_PITCH_R, JOINT_THUMB_PIP_PITCH_R, JOINT_PALM_AXIAL_R]
HAND_L_JOINT_ID_LIST = [JOINT_INDEX_MCP_PITCH_L, JOINT_INDEX_PIP_PITCH_L, JOINT_MIDDLE_MCP_PITCH_L, JOINT_MIDDLE_PIP_PITCH_L, JOINT_THUMB_MCP_PITCH_L, JOINT_THUMB_PIP_PITCH_L, JOINT_PALM_AXIAL_L]

LEG_JOINT_ID_LIST   =  LEG_R_JOINT_ID_LIST +  LEG_L_JOINT_ID_LIST
ARM_JOINT_ID_LIST   =  ARM_R_JOINT_ID_LIST +  ARM_L_JOINT_ID_LIST
HAND_JOINT_ID_LIST  = HAND_R_JOINT_ID_LIST + HAND_L_JOINT_ID_LIST
ROBOT_JOINT_ID_LIST =   BASE_JOINT_ID_LIST +    LEG_JOINT_ID_LIST + ARM_JOINT_ID_LIST + HEAD_JOINT_ID_LIST + HAND_JOINT_ID_LIST

for joint in ROBOT_JOINT_ID_LIST:
    JOINT[JOINT_ID][joint] = joint

# ------------------------------
# JOINT POSITION LIMIT
# ------------------------------
JOINT[JOINT_POSITION_LIMIT][JOINT_BASE] = -np.inf, np.inf

JOINT[JOINT_POSITION_LIMIT][JOINT_HIP_YAW_R]     = [-1.4, +1.4]  #  80 deg
JOINT[JOINT_POSITION_LIMIT][JOINT_HIP_ROLL_R]    = [-1.4, +1.4]  #  80 deg
JOINT[JOINT_POSITION_LIMIT][JOINT_HIP_PITCH_R]   = [-1.4, +1.4]  #  80 deg
JOINT[JOINT_POSITION_LIMIT][JOINT_KNEE_PITCH_R]  = [-2.4, -0.1]  # 140 deg
JOINT[JOINT_POSITION_LIMIT][JOINT_ANKLE_PITCH_R] = [-1.0, +1.5]  #  60 deg
JOINT[JOINT_POSITION_LIMIT][JOINT_ANKLE_ROLL_R]  = [-0.3, +0.3]  #  30 deg

JOINT[JOINT_POSITION_LIMIT][JOINT_HIP_YAW_L]     = JOINT[JOINT_POSITION_LIMIT][JOINT_HIP_YAW_R]
JOINT[JOINT_POSITION_LIMIT][JOINT_HIP_ROLL_L]    = JOINT[JOINT_POSITION_LIMIT][JOINT_HIP_ROLL_R]
JOINT[JOINT_POSITION_LIMIT][JOINT_HIP_PITCH_L]   = JOINT[JOINT_POSITION_LIMIT][JOINT_HIP_PITCH_R]
JOINT[JOINT_POSITION_LIMIT][JOINT_KNEE_PITCH_L]  = JOINT[JOINT_POSITION_LIMIT][JOINT_KNEE_PITCH_R]
JOINT[JOINT_POSITION_LIMIT][JOINT_ANKLE_PITCH_L] = JOINT[JOINT_POSITION_LIMIT][JOINT_ANKLE_PITCH_R]
JOINT[JOINT_POSITION_LIMIT][JOINT_ANKLE_ROLL_L]  = JOINT[JOINT_POSITION_LIMIT][JOINT_ANKLE_ROLL_R]

JOINT[JOINT_POSITION_LIMIT][JOINT_SHOULDER_PITCH_R] = [  -PI,   PI]
JOINT[JOINT_POSITION_LIMIT][JOINT_SHOULDER_ROLL_R]  = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_SHOULDER_YAW_R]   = [  -PI,   PI]
JOINT[JOINT_POSITION_LIMIT][JOINT_ELBOW_PITCH_R]    = [  -PI,   PI]
JOINT[JOINT_POSITION_LIMIT][JOINT_ELBOW_YAW_R]      = [  -PI,   PI]
JOINT[JOINT_POSITION_LIMIT][JOINT_WRIST_PITCH_R]    = [  -PI,   PI]
JOINT[JOINT_POSITION_LIMIT][JOINT_WRIST_YAW_R]      = [  -PI,   PI]

JOINT[JOINT_POSITION_LIMIT][JOINT_SHOULDER_PITCH_L] = [  -PI,   PI]
JOINT[JOINT_POSITION_LIMIT][JOINT_SHOULDER_ROLL_L]  = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_SHOULDER_YAW_L]   = [  -PI,   PI]
JOINT[JOINT_POSITION_LIMIT][JOINT_ELBOW_PITCH_L]    = [  -PI,   PI]
JOINT[JOINT_POSITION_LIMIT][JOINT_ELBOW_YAW_L]      = [  -PI,   PI]
JOINT[JOINT_POSITION_LIMIT][JOINT_WRIST_PITCH_L]    = [  -PI,   PI]
JOINT[JOINT_POSITION_LIMIT][JOINT_WRIST_YAW_L]      = [  -PI,   PI]

JOINT[JOINT_POSITION_LIMIT][JOINT_HEAD_YAW]   = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_HEAD_PITCH] = [-PI_2, PI_2]

JOINT[JOINT_POSITION_LIMIT][JOINT_INDEX_MCP_PITCH_R]  = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_INDEX_PIP_PITCH_R]  = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_MIDDLE_MCP_PITCH_R] = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_MIDDLE_PIP_PITCH_R] = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_THUMB_MCP_PITCH_R]  = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_THUMB_PIP_PITCH_R]  = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_PALM_AXIAL_R]       = [-PI_2, PI_2]

JOINT[JOINT_POSITION_LIMIT][JOINT_INDEX_MCP_PITCH_L]  = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_INDEX_PIP_PITCH_L]  = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_MIDDLE_MCP_PITCH_L] = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_MIDDLE_PIP_PITCH_L] = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_THUMB_MCP_PITCH_L]  = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_THUMB_PIP_PITCH_L]  = [-PI_2, PI_2]
JOINT[JOINT_POSITION_LIMIT][JOINT_PALM_AXIAL_L]       = [-PI_2, PI_2]

# ==================================================
# ==================================================
# LINK INFO
# ==================================================
# ==================================================
LINK = defaultdict(lambda: defaultdict(int))

# ------------------------------
# LINK PROPERTY
# ------------------------------
LINK_ID                    = 0
LINK_MASS                  = 1
LINK_INERTIA               = 2  # about CoM
LINK_COM_VECTOR            = 3  # relative to self frame
LINK_SPATIAL_INERTIA       = 4  # about self frame
LINK_POSITION_VECTOR       = 5  # relative to parent frame
LINK_ROTATION_VECTOR       = 6  # relative to parent frame
LINK_ROTATION_MATRIX       = 7  # relative to parent frame
LINK_TRANSFORMATION_MATRIX = 8  # relative to parent frame

LINK_BASE    = 0

LINK_HIP_R   = 1
LINK_ABAD_R  = 2
LINK_FEMUR_R = 3
LINK_TIBIA_R = 4
LINK_ANKLE_R = 5
LINK_FOOT_R  = 6

LINK_HIP_L   = 7
LINK_ABAD_L  = 8
LINK_FEMUR_L = 9
LINK_TIBIA_L = 10
LINK_ANKLE_L = 11
LINK_FOOT_L  = 12

LINK_UPPERSHOULDER_R = 13
LINK_LOWERSHOULDER_R = 14
LINK_UPPERARM_R      = 15
LINK_ELBOW_R         = 16
LINK_FOREARM_R       = 17
LINK_UPPERWRIST_R    = 18
LINK_LOWERWRIST_R    = 19

LINK_UPPERSHOULDER_L = 20
LINK_LOWERSHOULDER_L = 21
LINK_UPPERARM_L      = 22
LINK_ELBOW_L         = 23
LINK_FOREARM_L       = 24
LINK_UPPERWRIST_L    = 25
LINK_LOWERWRIST_L    = 26

LINK_NECK = 27
LINK_HEAD = 28

# ------------------------------
# LINK ID
# ------------------------------
BASE_LINK_ID_LIST  = [LINK_BASE]      
LEG_R_LINK_ID_LIST = [LINK_HIP_R, LINK_ABAD_R, LINK_FEMUR_R, LINK_TIBIA_R, LINK_ANKLE_R, LINK_FOOT_R]
LEG_L_LINK_ID_LIST = [LINK_HIP_L, LINK_ABAD_L, LINK_FEMUR_L, LINK_TIBIA_L, LINK_ANKLE_L, LINK_FOOT_L]
ARM_R_LINK_ID_LIST = [LINK_UPPERSHOULDER_R, LINK_LOWERSHOULDER_R, LINK_UPPERARM_R, LINK_ELBOW_R, LINK_FOREARM_R, LINK_UPPERWRIST_R, LINK_LOWERWRIST_R]
ARM_L_LINK_ID_LIST = [LINK_UPPERSHOULDER_L, LINK_LOWERSHOULDER_L, LINK_UPPERARM_L, LINK_ELBOW_L, LINK_FOREARM_L, LINK_UPPERWRIST_L, LINK_LOWERWRIST_L]
HEAD_LINK_ID_LIST  = [LINK_NECK, LINK_HEAD]

LEG_LINK_ID_LIST   = LEG_R_LINK_ID_LIST + LEG_L_LINK_ID_LIST
ARM_LINK_ID_LIST   = ARM_R_LINK_ID_LIST + ARM_L_LINK_ID_LIST
ROBOT_LINK_ID_LIST = BASE_LINK_ID_LIST + LEG_LINK_ID_LIST + ARM_LINK_ID_LIST + HEAD_LINK_ID_LIST

for link in ROBOT_LINK_ID_LIST:
    LINK[LINK_ID][link] = link

# ------------------------------
# LINK MASS
# ------------------------------
LINK[LINK_MASS][LINK_BASE] = 12.729530

LINK[LINK_MASS][LINK_HIP_R]   = 0.832640
LINK[LINK_MASS][LINK_ABAD_R]  = 2.818000
LINK[LINK_MASS][LINK_FEMUR_R] = 3.753930
LINK[LINK_MASS][LINK_TIBIA_R] = 1.277160
LINK[LINK_MASS][LINK_ANKLE_R] = 0.010000
LINK[LINK_MASS][LINK_FOOT_R]  = 0.416280

LINK[LINK_MASS][LINK_HIP_L]   = 0.832640
LINK[LINK_MASS][LINK_ABAD_L]  = 2.818000
LINK[LINK_MASS][LINK_FEMUR_L] = 3.753930
LINK[LINK_MASS][LINK_TIBIA_L] = 1.277160
LINK[LINK_MASS][LINK_ANKLE_L] = 0.010000
LINK[LINK_MASS][LINK_FOOT_L]  = 0.416280

LINK[LINK_MASS][LINK_UPPERSHOULDER_R] = 0.436110
LINK[LINK_MASS][LINK_LOWERSHOULDER_R] = 0.357980
LINK[LINK_MASS][LINK_UPPERARM_R]      = 0.515440
LINK[LINK_MASS][LINK_ELBOW_R]         = 0.378540
LINK[LINK_MASS][LINK_FOREARM_R]       = 0.476940	
LINK[LINK_MASS][LINK_UPPERWRIST_R]    = 0.370040
LINK[LINK_MASS][LINK_LOWERWRIST_R]    = 0.420000

LINK[LINK_MASS][LINK_UPPERSHOULDER_L] = 0.436110
LINK[LINK_MASS][LINK_LOWERSHOULDER_L] = 0.357980
LINK[LINK_MASS][LINK_UPPERARM_L]      = 0.515440
LINK[LINK_MASS][LINK_ELBOW_L]         = 0.378540
LINK[LINK_MASS][LINK_FOREARM_L]       = 0.476940	
LINK[LINK_MASS][LINK_UPPERWRIST_L]    = 0.370040
LINK[LINK_MASS][LINK_LOWERWRIST_L]    = 0.420000

LINK[LINK_MASS][LINK_NECK] = 0.340440
LINK[LINK_MASS][LINK_HEAD] = 0.673670

BASE_MASS  = 0.
LEG_R_MASS = 0.
LEG_L_MASS = 0.
ARM_R_MASS = 0.
ARM_L_MASS = 0.
HEAD_MASS  = 0.
for link in BASE_LINK_ID_LIST:
    BASE_MASS += LINK[LINK_MASS][link]
for link in LEG_R_LINK_ID_LIST:
    LEG_R_MASS += LINK[LINK_MASS][link]
for link in LEG_L_LINK_ID_LIST:
    LEG_L_MASS += LINK[LINK_MASS][link]
for link in ARM_R_LINK_ID_LIST:
    ARM_R_MASS += LINK[LINK_MASS][link]
for link in ARM_L_LINK_ID_LIST:
    ARM_L_MASS += LINK[LINK_MASS][link]
for link in HEAD_LINK_ID_LIST:
    HEAD_MASS += LINK[LINK_MASS][link]
ROBOT_MASS = BASE_MASS + LEG_R_MASS + LEG_L_MASS + ARM_R_MASS + ARM_L_MASS + HEAD_MASS
# print(ROBOT_MASS)

# ------------------------------
# LINK INERTIA
# ------------------------------
LINK[LINK_INERTIA][LINK_BASE]    = np.array([[ 0.257062, -0.000133,  0.031411],
                                             [-0.000133,  0.316902,  0.000321],
                                             [ 0.031411,  0.000321,  0.173619]])

LINK[LINK_INERTIA][LINK_HIP_R]   = np.array([[ 0.001900,        0.,        0.],
                                             [       0.,  0.001240,        0.],
                                             [       0.,        0.,  0.000940]])
LINK[LINK_INERTIA][LINK_ABAD_R]  = np.array([[ 0.014420,        0., -0.000020],
                                             [       0.,  0.007910,        0.],
                                             [-0.000020,        0.,  0.007200]])
LINK[LINK_INERTIA][LINK_FEMUR_R] = np.array([[ 0.011720,  0.001570,  0.000980],
                                             [ 0.001570,  0.070270,  0.000140],
                                             [ 0.000980,  0.000140,  0.075950]])
LINK[LINK_INERTIA][LINK_TIBIA_R] = np.array([[ 0.000910,  0.000690, -0.000010],
                                             [ 0.000690,  0.023230,        0.],
                                             [-0.000010,        0.,  0.023180]])
LINK[LINK_INERTIA][LINK_ANKLE_R] = np.array([[ 0.000100,        0.,        0.],
                                             [       0.,  0.000100,        0.],
                                             [       0.,        0.,  0.000100]])
LINK[LINK_INERTIA][LINK_FOOT_R]  = np.array([[ 0.002594,        0.,  0.000725],
                                             [       0.,  0.002998,  0.000001],
                                             [ 0.000725,  0.000001,  0.000722]])

LINK[LINK_INERTIA][LINK_HIP_L]   = np.array([[ 0.001900,        0.,        0.],
                                             [       0.,  0.001240,        0.],
                                             [       0.,        0.,  0.000940]])
LINK[LINK_INERTIA][LINK_ABAD_L]  = np.array([[ 0.014420,        0.,  0.000020],
                                             [       0.,  0.007910,        0.],
                                             [ 0.000020,        0.,  0.007200]])
LINK[LINK_INERTIA][LINK_FEMUR_L] = np.array([[ 0.011720,  0.001570, -0.000980],
                                             [ 0.001570,  0.070270, -0.000140],
                                             [-0.000980, -0.000140,  0.075950]])
LINK[LINK_INERTIA][LINK_TIBIA_L] = np.array([[ 0.000910,  0.000690, -0.000010],
                                             [ 0.000690,  0.023230,        0.],
                                             [-0.000010,        0.,  0.023180]])
LINK[LINK_INERTIA][LINK_ANKLE_L] = np.array([[ 0.000100,        0.,        0.],
                                             [       0.,  0.000100,        0.],
                                             [       0.,        0.,  0.000100]])
LINK[LINK_INERTIA][LINK_FOOT_L]  = np.array([[ 0.002594,        0.,  0.000725],
                                             [       0.,  0.002998,  0.000001],
                                             [ 0.000725,  0.000001,  0.000722]])

LINK[LINK_INERTIA][LINK_UPPERSHOULDER_R] = np.array([[ 0.000386,        0.,        0.],
                                                     [       0.,  0.000304,  0.000006],
                                                     [       0.,  0.000006,  0.000235]])
LINK[LINK_INERTIA][LINK_LOWERSHOULDER_R] = np.array([[ 0.000261, -0.000002,        0.],
                                                     [-0.000002,  0.001315,        0.],
                                                     [       0.,        0.,  0.001242]])
LINK[LINK_INERTIA][LINK_UPPERARM_R]      = np.array([[ 0.021672,  0.000010, -0.000176],
                                                     [ 0.000010,  0.021509, -0.001197],
                                                     [-0.000176, -0.001197,  0.000366]])
LINK[LINK_INERTIA][LINK_ELBOW_R]         = np.array([[ 0.000369, -0.000308,        0.],
                                                     [-0.000308,  0.001431,        0.],
                                                     [       0.,        0.,  0.001432]])
LINK[LINK_INERTIA][LINK_FOREARM_R]       = np.array([[ 0.000684,        0.,  0.000005],
                                                     [       0.,  0.000601,        0.],
                                                     [ 0.000005,        0.,  0.000265]])
LINK[LINK_INERTIA][LINK_UPPERWRIST_R]    = np.array([[ 0.000277, -0.000004,  0.000003],
                                                     [-0.000004,  0.001413,        0.],
                                                     [ 0.000003,        0.,  0.001337]])
LINK[LINK_INERTIA][LINK_LOWERWRIST_R]    = np.array([[ 0.000100,        0.,        0.],
                                                     [       0.,  0.000100,        0.],
                                                     [       0.,        0.,  0.000100]])

LINK[LINK_INERTIA][LINK_UPPERSHOULDER_L] = np.array([[ 0.000386,        0.,        0.],
                                                     [       0.,  0.000304, -0.000006],
                                                     [       0., -0.000006,  0.000235]])
LINK[LINK_INERTIA][LINK_LOWERSHOULDER_L] = np.array([[ 0.000261,  0.000002,        0.],
                                                     [ 0.000002,  0.001315,        0.],
                                                     [       0.,        0.,  0.001242]])
LINK[LINK_INERTIA][LINK_UPPERARM_L]      = np.array([[ 0.021672, -0.000010, -0.000176],
                                                     [-0.000010,  0.021509,  0.001197],
                                                     [-0.000176,  0.001197,  0.000366]])
LINK[LINK_INERTIA][LINK_ELBOW_L]         = np.array([[ 0.000369,  0.000308,        0.],
                                                     [ 0.000308,  0.001431,        0.],
                                                     [       0.,        0.,  0.001432]])
LINK[LINK_INERTIA][LINK_FOREARM_L]       = np.array([[ 0.000684,        0., -0.000005],
                                                     [       0.,  0.000601,        0.],
                                                     [-0.000005,        0.,  0.000265]])
LINK[LINK_INERTIA][LINK_UPPERWRIST_L]    = np.array([[ 0.000277,  0.000004,  0.000003],
                                                     [ 0.000004,  0.001413,        0.],
                                                     [ 0.000003,        0.,  0.001337]])
LINK[LINK_INERTIA][LINK_LOWERWRIST_L]    = np.array([[ 0.000100,        0.,        0.],
                                                     [       0.,  0.000100,        0.],
                                                     [       0.,        0.,  0.000100]])

LINK[LINK_INERTIA][LINK_NECK] = np.array([[ 0.000199,  0.000016, -0.000029],
                                          [ 0.000016,  0.000274, -0.000003],
                                          [-0.000029, -0.000003,  0.000167]])
LINK[LINK_INERTIA][LINK_HEAD] = np.array([[ 0.004633,  0.000093,  0.000261],
                                          [ 0.000093,  0.003171,  0.000001],
                                          [ 0.000261,  0.000001,  0.004426]])

# ------------------------------
# LINK COM VECTOR
# ------------------------------
LINK[LINK_COM_VECTOR][LINK_BASE]    = np.array([-0.081710,        0., -0.019790])

LINK[LINK_COM_VECTOR][LINK_HIP_R]   = np.array([ 0.003980, -0.000060,  0.006360])
LINK[LINK_COM_VECTOR][LINK_ABAD_R]  = np.array([-0.002000,  0.001470,  0.004040])
LINK[LINK_COM_VECTOR][LINK_FEMUR_R] = np.array([ 0.061120,  0.001770,  0.020020])
LINK[LINK_COM_VECTOR][LINK_TIBIA_R] = np.array([ 0.096780,  0.006940, -0.000060])
LINK[LINK_COM_VECTOR][LINK_ANKLE_R] = np.array([       0.,        0.,        0.])
LINK[LINK_COM_VECTOR][LINK_FOOT_R]  = np.array([ 0.033700,  0.000050,  0.046840])

LINK[LINK_COM_VECTOR][LINK_HIP_L]   = np.array([ 0.003980, -0.000060,  0.006360])
LINK[LINK_COM_VECTOR][LINK_ABAD_L]  = np.array([ 0.002000,  0.001470,  0.004040])
LINK[LINK_COM_VECTOR][LINK_FEMUR_L] = np.array([ 0.061120,  0.001770, -0.020020])
LINK[LINK_COM_VECTOR][LINK_TIBIA_L] = np.array([ 0.096780,  0.006940, -0.000060])
LINK[LINK_COM_VECTOR][LINK_ANKLE_L] = np.array([       0.,        0.,        0.])
LINK[LINK_COM_VECTOR][LINK_FOOT_L]  = np.array([ 0.033700,  0.000050,  0.046840])

LINK[LINK_COM_VECTOR][LINK_UPPERSHOULDER_R] = np.array([ 0.001580,  0.001230, -0.006740])
LINK[LINK_COM_VECTOR][LINK_LOWERSHOULDER_R] = np.array([ 0.053500, -0.000060,  0.000020])
LINK[LINK_COM_VECTOR][LINK_UPPERARM_R]      = np.array([-0.001520, -0.010400,  0.197300])
LINK[LINK_COM_VECTOR][LINK_ELBOW_R]         = np.array([ 0.053870, -0.014410,  0.000010])
LINK[LINK_COM_VECTOR][LINK_FOREARM_R]       = np.array([-0.001640,  0.000020, -0.014070])
LINK[LINK_COM_VECTOR][LINK_UPPERWRIST_R]    = np.array([ 0.054630, -0.000130,  0.000090])
LINK[LINK_COM_VECTOR][LINK_LOWERWRIST_R]    = np.array([       0.,  0.000002,  0.000108])

LINK[LINK_COM_VECTOR][LINK_UPPERSHOULDER_L] = np.array([ 0.001580,  0.001230,  0.006740])
LINK[LINK_COM_VECTOR][LINK_LOWERSHOULDER_L] = np.array([ 0.053500,  0.000060, -0.000020])
LINK[LINK_COM_VECTOR][LINK_UPPERARM_L]      = np.array([-0.001520,  0.010420,  0.197300])
LINK[LINK_COM_VECTOR][LINK_ELBOW_L]         = np.array([ 0.053870,  0.014410, -0.000010])
LINK[LINK_COM_VECTOR][LINK_FOREARM_L]       = np.array([ 0.001640,  0.000020, -0.014070])
LINK[LINK_COM_VECTOR][LINK_UPPERWRIST_L]    = np.array([ 0.054630,  0.000130,  0.000090])
LINK[LINK_COM_VECTOR][LINK_LOWERWRIST_L]    = np.array([       0., -0.000002,  0.000108])

LINK[LINK_COM_VECTOR][LINK_NECK] = np.array([       0.,  0.000170, -0.004890])
LINK[LINK_COM_VECTOR][LINK_HEAD] = np.array([ 0.000940,  0.063100, -0.000010])

# ------------------------------
# LINK SPATIAL INERTIA
# ------------------------------
for link in ROBOT_LINK_ID_LIST:
    # c_hat = np.array([[                             0., -LINK[LINK_COM_VECTOR][link][2],  LINK[LINK_COM_VECTOR][link][1]],
    #                   [ LINK[LINK_COM_VECTOR][link][2],                              0., -LINK[LINK_COM_VECTOR][link][0]],
    #                   [-LINK[LINK_COM_VECTOR][link][1],  LINK[LINK_COM_VECTOR][link][0],                              0.]])
    c_hat = MF.hat(LINK[LINK_COM_VECTOR][link])
    LINK[LINK_SPATIAL_INERTIA][link] = np.zeros((6, 6))
    LINK[LINK_SPATIAL_INERTIA][link][0:3, 0:3] = LINK[LINK_INERTIA][link] + LINK[LINK_MASS][link] * c_hat @ c_hat.T
    LINK[LINK_SPATIAL_INERTIA][link][0:3, 3:6] = LINK[LINK_MASS][link] * c_hat
    LINK[LINK_SPATIAL_INERTIA][link][3:6, 0:3] = LINK[LINK_MASS][link] * c_hat.T
    LINK[LINK_SPATIAL_INERTIA][link][3:6, 3:6] = LINK[LINK_MASS][link] * np.eye(3)

# ------------------------------
# LINK POSITION VECTOR
# ------------------------------
LINK[LINK_POSITION_VECTOR][LINK_BASE] = np.zeros(3)

LINK[LINK_POSITION_VECTOR][LINK_HIP_R]   = np.array([-0.183378, -0.062500, -0.288378])
LINK[LINK_POSITION_VECTOR][LINK_ABAD_R]  = np.array([ 0.141000,        0.,        0.])
LINK[LINK_POSITION_VECTOR][LINK_FEMUR_R] = np.array([ 0.029000,        0.,        0.])
LINK[LINK_POSITION_VECTOR][LINK_TIBIA_R] = np.array([ 0.375000,        0.,        0.])
LINK[LINK_POSITION_VECTOR][LINK_ANKLE_R] = np.array([ 0.375000,        0.,        0.])
LINK[LINK_POSITION_VECTOR][LINK_FOOT_R]  = np.array([       0.,        0.,        0.])

LINK[LINK_POSITION_VECTOR][LINK_HIP_L]   = np.array([-0.183378,  0.062500, -0.288378])
LINK[LINK_POSITION_VECTOR][LINK_ABAD_L]  = np.array([ 0.141000,        0.,        0.])
LINK[LINK_POSITION_VECTOR][LINK_FEMUR_L] = np.array([-0.029000,        0.,        0.])
LINK[LINK_POSITION_VECTOR][LINK_TIBIA_L] = np.array([ 0.375000,        0.,        0.])
LINK[LINK_POSITION_VECTOR][LINK_ANKLE_L] = np.array([ 0.375000,        0.,        0.])
LINK[LINK_POSITION_VECTOR][LINK_FOOT_L]  = np.array([       0.,        0.,        0.])

LINK[LINK_POSITION_VECTOR][LINK_UPPERSHOULDER_R] = np.array([-0.065085, -0.156700,  0.113475])
LINK[LINK_POSITION_VECTOR][LINK_LOWERSHOULDER_R] = np.array([       0.,        0.,        0.])
LINK[LINK_POSITION_VECTOR][LINK_UPPERARM_R]      = np.array([       0.,        0.,        0.])
LINK[LINK_POSITION_VECTOR][LINK_ELBOW_R]         = np.array([       0., -0.015000,  0.224950])
LINK[LINK_POSITION_VECTOR][LINK_FOREARM_R]       = np.array([ 0.154950, -0.015000,        0.])
LINK[LINK_POSITION_VECTOR][LINK_UPPERWRIST_R]    = np.array([       0.,        0.,        0.])
LINK[LINK_POSITION_VECTOR][LINK_LOWERWRIST_R]    = np.array([       0.,        0.,        0.])

LINK[LINK_POSITION_VECTOR][LINK_UPPERSHOULDER_L] = np.array([-0.065085,  0.156700,  0.113475])
LINK[LINK_POSITION_VECTOR][LINK_LOWERSHOULDER_L] = np.array([       0.,        0.,        0.])
LINK[LINK_POSITION_VECTOR][LINK_UPPERARM_L]      = np.array([       0.,        0.,        0.])
LINK[LINK_POSITION_VECTOR][LINK_ELBOW_L]         = np.array([       0.,  0.015000,  0.224950])
LINK[LINK_POSITION_VECTOR][LINK_FOREARM_L]       = np.array([ 0.154950,  0.015000,        0.])
LINK[LINK_POSITION_VECTOR][LINK_UPPERWRIST_L]    = np.array([       0.,        0.,        0.])
LINK[LINK_POSITION_VECTOR][LINK_LOWERWRIST_L]    = np.array([       0.,        0.,        0.])

LINK[LINK_POSITION_VECTOR][LINK_NECK] = np.array([-0.069374,        0.,  0.230836])
LINK[LINK_POSITION_VECTOR][LINK_HEAD] = np.array([       0.,        0.,        0.])

# ------------------------------
# LINK ROTATION VECTOR
# ------------------------------
LINK[LINK_ROTATION_VECTOR][LINK_BASE] = np.zeros(3)

LINK[LINK_ROTATION_VECTOR][LINK_HIP_R]   = np.array([   0.,  PI_4,    0.])
LINK[LINK_ROTATION_VECTOR][LINK_ABAD_R]  = np.array([ PI_2,    0., -PI_2])
LINK[LINK_ROTATION_VECTOR][LINK_FEMUR_R] = np.array([-PI_2,  PI_4, -PI_2])
LINK[LINK_ROTATION_VECTOR][LINK_TIBIA_R] = np.array([   0.,    0.,    0.])
LINK[LINK_ROTATION_VECTOR][LINK_ANKLE_R] = np.array([   0.,    0.,    0.])
LINK[LINK_ROTATION_VECTOR][LINK_FOOT_R]  = np.array([-PI_2,    0.,    0.])

LINK[LINK_ROTATION_VECTOR][LINK_HIP_L]   = np.array([   0.,  PI_4,    0.])
LINK[LINK_ROTATION_VECTOR][LINK_ABAD_L]  = np.array([ PI_2,    0., -PI_2])
LINK[LINK_ROTATION_VECTOR][LINK_FEMUR_L] = np.array([-PI_2,  PI_4, -PI_2])
LINK[LINK_ROTATION_VECTOR][LINK_TIBIA_L] = np.array([   0.,    0.,    0.])
LINK[LINK_ROTATION_VECTOR][LINK_ANKLE_L] = np.array([   0.,    0.,    0.])
LINK[LINK_ROTATION_VECTOR][LINK_FOOT_L]  = np.array([-PI_2,    0.,    0.])

LINK[LINK_ROTATION_VECTOR][LINK_UPPERSHOULDER_R] = np.array([ PI_2,    0.,  0.])
LINK[LINK_ROTATION_VECTOR][LINK_LOWERSHOULDER_R] = np.array([   0., -PI_2,  PI])
LINK[LINK_ROTATION_VECTOR][LINK_UPPERARM_R]      = np.array([   0., -PI_2,  PI])
LINK[LINK_ROTATION_VECTOR][LINK_ELBOW_R]         = np.array([   0., -PI_2,  PI])
LINK[LINK_ROTATION_VECTOR][LINK_FOREARM_R]       = np.array([   0., -PI_2,  PI])
LINK[LINK_ROTATION_VECTOR][LINK_UPPERWRIST_R]    = np.array([   0., -PI_2,  PI])
LINK[LINK_ROTATION_VECTOR][LINK_LOWERWRIST_R]    = np.array([   0., -PI_2,  PI])

LINK[LINK_ROTATION_VECTOR][LINK_UPPERSHOULDER_L] = np.array([ PI_2,    0.,  0.])
LINK[LINK_ROTATION_VECTOR][LINK_LOWERSHOULDER_L] = np.array([   0.,  PI_2,  0.])
LINK[LINK_ROTATION_VECTOR][LINK_UPPERARM_L]      = np.array([   0., -PI_2,  PI])
LINK[LINK_ROTATION_VECTOR][LINK_ELBOW_L]         = np.array([   0., -PI_2,  PI])
LINK[LINK_ROTATION_VECTOR][LINK_FOREARM_L]       = np.array([   0., -PI_2,  PI])
LINK[LINK_ROTATION_VECTOR][LINK_UPPERWRIST_L]    = np.array([   0., -PI_2,  PI])
LINK[LINK_ROTATION_VECTOR][LINK_LOWERWRIST_L]    = np.array([   0., -PI_2,  PI])

LINK[LINK_ROTATION_VECTOR][LINK_NECK] = np.array([   0.,  0.,  0.])
LINK[LINK_ROTATION_VECTOR][LINK_HEAD] = np.array([ PI_2,  0.,  0.])

# ------------------------------
# LINK ROTATION MATRIX
# ------------------------------
for link in ROBOT_LINK_ID_LIST:
    LINK[LINK_ROTATION_MATRIX][link] = MF.Rz(LINK[LINK_ROTATION_VECTOR][link][2]) @ MF.Ry(LINK[LINK_ROTATION_VECTOR][link][1]) @ MF.Rx(LINK[LINK_ROTATION_VECTOR][link][0])

# ------------------------------
# LINK TRANSFORMATION MATRIX
# ------------------------------
for link in ROBOT_LINK_ID_LIST:
    LINK[LINK_TRANSFORMATION_MATRIX][link] = np.eye(4)
    LINK[LINK_TRANSFORMATION_MATRIX][link][0:3, 0:3] = np.array(LINK[LINK_ROTATION_MATRIX][link])
    LINK[LINK_TRANSFORMATION_MATRIX][link][0:3,   3] = np.array(LINK[LINK_POSITION_VECTOR][link])
