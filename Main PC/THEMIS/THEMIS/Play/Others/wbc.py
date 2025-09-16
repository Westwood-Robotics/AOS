#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "August 25, 2025"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
THEMIS WBC API
ATTENTION: do not make changes to this file unless you know what you are doing :)
'''

import Startup.memory_manager as MM
from Setting.Macros.bear_macros import *
from Setting.Macros.model_macros import *


def get_imu_states():
    """
    Get imu states

    RETURN:
    a (1 x 3) - linear acceleration
    w (1 x 3) - angular rate
    R (3 x 3) - rotation matrix
    """
    data = MM.SENSE_STATE.get()
    a = data['imu_acceleration']
    w = data['imu_angular_rate']
    R = data['imu_rotation_matrix']
    return a, w, R

def get_joint_states(chain):
    """
    Get joint states

    ARGUMENT:
    chain (  int) - limb chain index, i.e., head (0), right leg (+1), left leg (-1), right arm (+2), left arm (-2), right hand (+3), left hand (-3)
    
    RETURN:
    q  (1 x n) - joint positions
    dq (1 x n) - joint velocities
    u  (1 x n) - joint torques
    t  (1 x n) - motor temperatures
    v  (1 x n) - motor voltages
    """
    if chain == 0:
        data = MM.HEAD_JOINT_STATE.get()
        t = data['bear_temperatures']
        v = data['bear_voltages']
    elif chain == +1:
        data = MM.RIGHT_LEG_JOINT_STATE.get()
        t = data['bear_temperatures']
        v = data['bear_voltages']
    elif chain == -1:
        data = MM.LEFT_LEG_JOINT_STATE.get()
        t = data['bear_temperatures']
        v = data['bear_voltages']
    elif chain == +2:
        data = MM.RIGHT_ARM_JOINT_STATE.get()
        t = data['bear_temperatures']
        v = data['bear_voltages']
    elif chain == -2:
        data = MM.LEFT_ARM_JOINT_STATE.get()
        t = data['bear_temperatures']
        v = data['bear_voltages']
    elif chain == +3:
        data = MM.RIGHT_HAND_JOINT_STATE.get()
        t = data['dxl_temperatures']
        v = data['dxl_voltages']
    elif chain == -3:
        data = MM.LEFT_HAND_JOINT_STATE.get()
        t = data['dxl_temperatures']
        v = data['dxl_voltages']

    q  = data['joint_positions']
    dq = data['joint_velocities']
    u  = data['joint_torques']

    return q, dq, u, t, v

def set_joint_states(chain, u, q, dq, kp, kd):
    """
    Set joint states

    ARGUMENT:
    chain (  int) - limb chain index, i.e., head (0), right leg (+1), left leg (-1), right arm (+2), left arm (-2), right hand (+3), left hand (-3)
    u     (1 x n) - joint torques
    q     (1 x n) - joint positions
    dq    (1 x n) - joint velocities
    kp    (1 x n) - motor p gains
    kd    (1 x n) - motor d gains
    """
    if chain == 0:
        command = {'goal_joint_positions':    np.array([q]),
                   'goal_joint_velocities':   np.array([dq]),
                   'goal_joint_torques':      np.array([u]),
                   'bear_enable_statuses':    np.array([1]*2),
                   'bear_operating_modes':    np.array([3]*2),
                   'bear_proportional_gains': np.array([kp]),
                   'bear_derivative_gains':   np.array([kd])}
        MM.HEAD_JOINT_COMMAND.set(command)
    elif chain == +1:
        command = {'goal_joint_positions':    np.array([q]),
                   'goal_joint_velocities':   np.array([dq]),
                   'goal_joint_torques':      np.array([u]),
                   'bear_enable_statuses':    np.array([1]*6),
                   'bear_operating_modes':    np.array([3]*6),
                   'bear_proportional_gains': np.array([kp]),
                   'bear_derivative_gains':   np.array([kd])}
        MM.RIGHT_LEG_JOINT_COMMAND.set(command)
    elif chain == -1:
        command = {'goal_joint_positions':    np.array([q]),
                   'goal_joint_velocities':   np.array([dq]),
                   'goal_joint_torques':      np.array([u]),
                   'bear_enable_statuses':    np.array([1]*6),
                   'bear_operating_modes':    np.array([3]*6),
                   'bear_proportional_gains': np.array([kp]),
                   'bear_derivative_gains':   np.array([kd])}
        MM.LEFT_LEG_JOINT_COMMAND.set(command)
    elif chain == +2:
        command = {'goal_joint_positions':    np.array([q]),
                   'goal_joint_velocities':   np.array([dq]),
                   'goal_joint_torques':      np.array([u]),
                   'bear_enable_statuses':    np.array([1]*7),
                   'bear_operating_modes':    np.array([3]*7),
                   'bear_proportional_gains': np.array([kp]),
                   'bear_derivative_gains':   np.array([kd])}
        MM.RIGHT_ARM_JOINT_COMMAND.set(command)
    elif chain == -2:
        command = {'goal_joint_positions':    np.array([q]),
                   'goal_joint_velocities':   np.array([dq]),
                   'goal_joint_torques':      np.array([u]),
                   'bear_enable_statuses':    np.array([1]*7),
                   'bear_operating_modes':    np.array([3]*7),
                   'bear_proportional_gains': np.array([kp]),
                   'bear_derivative_gains':   np.array([kd])}
        MM.LEFT_ARM_JOINT_COMMAND.set(command)
    elif chain == +3:
        command = {'goal_joint_positions':   np.array([q]),
                   'goal_joint_velocities':  np.array([dq]),
                #    'goal_joint_torques':     np.array([u]),
                   'dxl_enable_statuses':    np.array([1]*7),
                   'dxl_operating_modes':    np.array([3]*7),
                   'dxl_proportional_gains': np.array([kp]),
                   'dxl_derivative_gains':   np.array([kd])}
        MM.RIGHT_HAND_JOINT_COMMAND.set(command)
    elif chain == -3:
        command = {'goal_joint_positions':   np.array([q]),
                   'goal_joint_velocities':  np.array([dq]),
                #    'goal_joint_torques':     np.array([u]),
                   'dxl_enable_statuses':    np.array([1]*7),
                   'dxl_operating_modes':    np.array([3]*7),
                   'dxl_proportional_gains': np.array([kp]),
                   'dxl_derivative_gains':   np.array([kd])}
        MM.LEFT_HAND_JOINT_COMMAND.set(command)

def disable_motors(chain):
    """
    Disable motors

    ARGUMENT:
    chain (int) - limb chain index, i.e., head (0), right leg (+1), left leg (-1), right arm (+2), left arm (-2), right hand (+3), left hand (-3)
    """
    if chain == 0:
        command = {'bear_enable_statuses': np.array([0]*2),}
        MM.HEAD_JOINT_COMMAND.set(command)
    elif chain == +1:
        command = {'bear_enable_statuses': np.array([0]*6),}
        MM.RIGHT_LEG_JOINT_COMMAND.set(command)
    elif chain == -1:
        command = {'bear_enable_statuses': np.array([0]*6),}
        MM.LEFT_LEG_JOINT_COMMAND.set(command)
    elif chain == +2:
        command = {'bear_enable_statuses': np.array([0]*7),}
        MM.RIGHT_ARM_JOINT_COMMAND.set(command)
    elif chain == -2:
        command = {'bear_enable_statuses': np.array([0]*7),}
        MM.LEFT_ARM_JOINT_COMMAND.set(command)
    elif chain == +3:
        command = {'dxl_enable_statuses': np.array([0]*7),}
        MM.RIGHT_HAND_JOINT_COMMAND.set(command)
    elif chain == -3:
        command = {'dxl_enable_statuses': np.array([0]*7),}
        MM.LEFT_HAND_JOINT_COMMAND.set(command)

def damping_motors(chain):
    """
    Damping motors

    ARGUMENT:
    chain (int) - limb chain index, i.e., head (0), right leg (+1), left leg (-1), right arm (+2), left arm (-2)
    """
    if chain == 0:
        command = {'bear_enable_statuses': np.array([3]*2),}
        MM.HEAD_JOINT_COMMAND.set(command)
    elif chain == +1:
        command = {'bear_enable_statuses': np.array([3]*6),}
        MM.RIGHT_LEG_JOINT_COMMAND.set(command)
    elif chain == -1:
        command = {'bear_enable_statuses': np.array([3]*6),}
        MM.LEFT_LEG_JOINT_COMMAND.set(command)
    elif chain == +2:
        command = {'bear_enable_statuses': np.array([3]*7),}
        MM.RIGHT_ARM_JOINT_COMMAND.set(command)
    elif chain == -2:
        command = {'bear_enable_statuses': np.array([3]*7),}
        MM.LEFT_ARM_JOINT_COMMAND.set(command)