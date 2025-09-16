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
Pre-generate the shared memory segments in advance
'''

import numpy as np
from Library.SHARED_MEMORY import Manager as shared_memory_manager


# Thread State
THREAD_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='THREAD_STATE', init=False)
THREAD_STATE.add_block(name='timestamp', data=np.zeros(1))

THREAD_STATE.add_block(name='simulation',     data=np.zeros(1))
THREAD_STATE.add_block(name='bear_right_leg', data=np.zeros(1))
THREAD_STATE.add_block(name='bear_left_leg',  data=np.zeros(1))
THREAD_STATE.add_block(name='bear_right_arm', data=np.zeros(1))
THREAD_STATE.add_block(name='bear_left_arm',  data=np.zeros(1))
THREAD_STATE.add_block(name='bear_head',      data=np.zeros(1))
THREAD_STATE.add_block(name='dxl_right_hand', data=np.zeros(1))
THREAD_STATE.add_block(name='dxl_left_hand',  data=np.zeros(1))
THREAD_STATE.add_block(name='sense',          data=np.zeros(1))
THREAD_STATE.add_block(name='estimation',     data=np.zeros(1))
THREAD_STATE.add_block(name='low_level',      data=np.zeros(1))
THREAD_STATE.add_block(name='high_level',     data=np.zeros(1))
THREAD_STATE.add_block(name='top_level',      data=np.zeros(1))
THREAD_STATE.add_block(name='battery',        data=np.zeros(1))
THREAD_STATE.add_block(name='auxiliary',      data=np.zeros(1))

# Thread Command
THREAD_COMMAND = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='THREAD_COMMAND', init=False)
THREAD_COMMAND.add_block(name='timestamp', data=np.zeros(1))

THREAD_COMMAND.add_block(name='simulation',     data=np.zeros(1))
THREAD_COMMAND.add_block(name='bear_right_leg', data=np.zeros(1))
THREAD_COMMAND.add_block(name='bear_left_leg',  data=np.zeros(1))
THREAD_COMMAND.add_block(name='bear_right_arm', data=np.zeros(1))
THREAD_COMMAND.add_block(name='bear_left_arm',  data=np.zeros(1))
THREAD_COMMAND.add_block(name='bear_head',      data=np.zeros(1))
THREAD_COMMAND.add_block(name='dxl_right_hand', data=np.zeros(1))
THREAD_COMMAND.add_block(name='dxl_left_hand',  data=np.zeros(1))
THREAD_COMMAND.add_block(name='sense',          data=np.zeros(1))
THREAD_COMMAND.add_block(name='estimation',     data=np.zeros(1))
THREAD_COMMAND.add_block(name='low_level',      data=np.zeros(1))
THREAD_COMMAND.add_block(name='high_level',     data=np.zeros(1))
THREAD_COMMAND.add_block(name='top_level',      data=np.zeros(1))
THREAD_COMMAND.add_block(name='battery',        data=np.zeros(1))
THREAD_COMMAND.add_block(name='auxiliary',      data=np.zeros(1))


# Simulation State
SIMULATION_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='SIMULATION_STATE', init=False)
SIMULATION_STATE.add_block(name='timestamp', data=np.zeros(1))

SIMULATION_STATE.add_block(name='time', data=np.zeros(1))


# Sense State
SENSE_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='SENSE_STATE', init=False)
SENSE_STATE.add_block(name='timestamp', data=np.zeros(1))

SENSE_STATE.add_block(name='imu_acceleration',    data=np.zeros(3))
SENSE_STATE.add_block(name='imu_angular_rate',    data=np.zeros(3))
SENSE_STATE.add_block(name='imu_rotation_matrix', data=np.eye(3))


# Battery State
BATTERY_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='BATTERY_STATE', init=False)
BATTERY_STATE.add_block(name='timestamp', data=np.zeros(1))

BATTERY_STATE.add_block(name='battery_statuses',       data=np.zeros(2))
BATTERY_STATE.add_block(name='error_statuses',         data=np.zeros(2))
BATTERY_STATE.add_block(name='battery_voltages',       data=np.zeros(2))
BATTERY_STATE.add_block(name='cell_voltages',          data=np.zeros((2, 8)))
BATTERY_STATE.add_block(name='dsg_statuses',           data=np.zeros(2))
BATTERY_STATE.add_block(name='chg_statuses',           data=np.zeros(2))
BATTERY_STATE.add_block(name='max_discharge_currents', data=np.zeros(2))
BATTERY_STATE.add_block(name='max_charge_currents',    data=np.zeros(2))
BATTERY_STATE.add_block(name='present_currents',       data=np.zeros(2))
BATTERY_STATE.add_block(name='temperatures',           data=np.zeros(2))


# Gamepad State
GAMEPAD_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='GAMEPAD_STATE', init=False)
GAMEPAD_STATE.add_block(name='timestamp', data=np.zeros(1))

GAMEPAD_STATE.add_block(name='U',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='D',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='L',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='R',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='A',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='B',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='X',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='Y',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LZ',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LS',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LS2', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LSP', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LSM', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RZ',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RS',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RS2', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RSP', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RSM', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='ST',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='BK',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='ALT', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='FN',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LX',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LY',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RX',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RY',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LSZ', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RSZ', data=np.zeros(1))

GAMEPAD_STATE.add_block(name='connection', data=np.zeros(1))


# Right Leg Joint State
RIGHT_LEG_JOINT_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='RIGHT_LEG_JOINT_STATE', init=False)
RIGHT_LEG_JOINT_STATE.add_block(name='timestamp', data=np.zeros(1))

RIGHT_LEG_JOINT_STATE.add_block(name='joint_positions',   data=np.zeros(6))
RIGHT_LEG_JOINT_STATE.add_block(name='joint_velocities',  data=np.zeros(6))
RIGHT_LEG_JOINT_STATE.add_block(name='joint_torques',     data=np.zeros(6))
RIGHT_LEG_JOINT_STATE.add_block(name='bear_temperatures', data=np.zeros(6))
RIGHT_LEG_JOINT_STATE.add_block(name='bear_voltages',     data=np.zeros(6))

# Right Leg Joint Command
RIGHT_LEG_JOINT_COMMAND = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='RIGHT_LEG_JOINT_COMMAND', init=False)
RIGHT_LEG_JOINT_COMMAND.add_block(name='timestamp', data=np.zeros(1))

RIGHT_LEG_JOINT_COMMAND.add_block(name='goal_joint_positions',    data=np.zeros(6))
RIGHT_LEG_JOINT_COMMAND.add_block(name='goal_joint_velocities',   data=np.zeros(6))
RIGHT_LEG_JOINT_COMMAND.add_block(name='goal_joint_torques',      data=np.zeros(6))
RIGHT_LEG_JOINT_COMMAND.add_block(name='bear_enable_statuses',    data=np.zeros(6))
RIGHT_LEG_JOINT_COMMAND.add_block(name='bear_operating_modes',    data=np.zeros(6))
RIGHT_LEG_JOINT_COMMAND.add_block(name='bear_proportional_gains', data=np.zeros(6))
RIGHT_LEG_JOINT_COMMAND.add_block(name='bear_derivative_gains',   data=np.zeros(6))


# Left Leg Joint State
LEFT_LEG_JOINT_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='LEFT_LEG_JOINT_STATE', init=False)
LEFT_LEG_JOINT_STATE.add_block(name='timestamp', data=np.zeros(1))

LEFT_LEG_JOINT_STATE.add_block(name='joint_positions',   data=np.zeros(6))
LEFT_LEG_JOINT_STATE.add_block(name='joint_velocities',  data=np.zeros(6))
LEFT_LEG_JOINT_STATE.add_block(name='joint_torques',     data=np.zeros(6))
LEFT_LEG_JOINT_STATE.add_block(name='bear_temperatures', data=np.zeros(6))
LEFT_LEG_JOINT_STATE.add_block(name='bear_voltages',     data=np.zeros(6))

# Left Leg Joint Command
LEFT_LEG_JOINT_COMMAND = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='LEFT_LEG_JOINT_COMMAND', init=False)
LEFT_LEG_JOINT_COMMAND.add_block(name='timestamp', data=np.zeros(1))

LEFT_LEG_JOINT_COMMAND.add_block(name='goal_joint_positions',    data=np.zeros(6))
LEFT_LEG_JOINT_COMMAND.add_block(name='goal_joint_velocities',   data=np.zeros(6))
LEFT_LEG_JOINT_COMMAND.add_block(name='goal_joint_torques',      data=np.zeros(6))
LEFT_LEG_JOINT_COMMAND.add_block(name='bear_enable_statuses',    data=np.zeros(6))
LEFT_LEG_JOINT_COMMAND.add_block(name='bear_operating_modes',    data=np.zeros(6))
LEFT_LEG_JOINT_COMMAND.add_block(name='bear_proportional_gains', data=np.zeros(6))
LEFT_LEG_JOINT_COMMAND.add_block(name='bear_derivative_gains',   data=np.zeros(6))


# Right Arm Joint State
RIGHT_ARM_JOINT_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='RIGHT_ARM_JOINT_STATE', init=False)
RIGHT_ARM_JOINT_STATE.add_block(name='timestamp', data=np.zeros(1))

RIGHT_ARM_JOINT_STATE.add_block(name='joint_positions',   data=np.zeros(7))
RIGHT_ARM_JOINT_STATE.add_block(name='joint_velocities',  data=np.zeros(7))
RIGHT_ARM_JOINT_STATE.add_block(name='joint_torques',     data=np.zeros(7))
RIGHT_ARM_JOINT_STATE.add_block(name='bear_temperatures', data=np.zeros(7))
RIGHT_ARM_JOINT_STATE.add_block(name='bear_voltages',     data=np.zeros(7))

# Right Arm Joint Command
RIGHT_ARM_JOINT_COMMAND = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='RIGHT_ARM_JOINT_COMMAND', init=False)
RIGHT_ARM_JOINT_COMMAND.add_block(name='timestamp', data=np.zeros(1))

RIGHT_ARM_JOINT_COMMAND.add_block(name='goal_joint_positions',    data=np.zeros(7))
RIGHT_ARM_JOINT_COMMAND.add_block(name='goal_joint_velocities',   data=np.zeros(7))
RIGHT_ARM_JOINT_COMMAND.add_block(name='goal_joint_torques',      data=np.zeros(7))
RIGHT_ARM_JOINT_COMMAND.add_block(name='bear_enable_statuses',    data=np.zeros(7))
RIGHT_ARM_JOINT_COMMAND.add_block(name='bear_operating_modes',    data=np.zeros(7))
RIGHT_ARM_JOINT_COMMAND.add_block(name='bear_proportional_gains', data=np.zeros(7))
RIGHT_ARM_JOINT_COMMAND.add_block(name='bear_derivative_gains',   data=np.zeros(7))


# Left Arm Joint State
LEFT_ARM_JOINT_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='LEFT_ARM_JOINT_STATE', init=False)
LEFT_ARM_JOINT_STATE.add_block(name='timestamp', data=np.zeros(1))

LEFT_ARM_JOINT_STATE.add_block(name='joint_positions',   data=np.zeros(7))
LEFT_ARM_JOINT_STATE.add_block(name='joint_velocities',  data=np.zeros(7))
LEFT_ARM_JOINT_STATE.add_block(name='joint_torques',     data=np.zeros(7))
LEFT_ARM_JOINT_STATE.add_block(name='bear_temperatures', data=np.zeros(7))
LEFT_ARM_JOINT_STATE.add_block(name='bear_voltages',     data=np.zeros(7))

# Left Arm Joint Command
LEFT_ARM_JOINT_COMMAND = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='LEFT_ARM_JOINT_COMMAND', init=False)
LEFT_ARM_JOINT_COMMAND.add_block(name='timestamp', data=np.zeros(1))

LEFT_ARM_JOINT_COMMAND.add_block(name='goal_joint_positions',    data=np.zeros(7))
LEFT_ARM_JOINT_COMMAND.add_block(name='goal_joint_velocities',   data=np.zeros(7))
LEFT_ARM_JOINT_COMMAND.add_block(name='goal_joint_torques',      data=np.zeros(7))
LEFT_ARM_JOINT_COMMAND.add_block(name='bear_enable_statuses',    data=np.zeros(7))
LEFT_ARM_JOINT_COMMAND.add_block(name='bear_operating_modes',    data=np.zeros(7))
LEFT_ARM_JOINT_COMMAND.add_block(name='bear_proportional_gains', data=np.zeros(7))
LEFT_ARM_JOINT_COMMAND.add_block(name='bear_derivative_gains',   data=np.zeros(7))


# Head Joint State
HEAD_JOINT_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='HEAD_JOINT_STATE', init=False)
HEAD_JOINT_STATE.add_block(name='timestamp', data=np.zeros(1))

HEAD_JOINT_STATE.add_block(name='joint_positions',   data=np.zeros(2))
HEAD_JOINT_STATE.add_block(name='joint_velocities',  data=np.zeros(2))
HEAD_JOINT_STATE.add_block(name='joint_torques',     data=np.zeros(2))
HEAD_JOINT_STATE.add_block(name='bear_temperatures', data=np.zeros(2))
HEAD_JOINT_STATE.add_block(name='bear_voltages',     data=np.zeros(2))

# Head Joint Command
HEAD_JOINT_COMMAND = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='HEAD_JOINT_COMMAND', init=False)
HEAD_JOINT_COMMAND.add_block(name='timestamp', data=np.zeros(1))

HEAD_JOINT_COMMAND.add_block(name='goal_joint_positions',    data=np.zeros(2))
HEAD_JOINT_COMMAND.add_block(name='goal_joint_velocities',   data=np.zeros(2))
HEAD_JOINT_COMMAND.add_block(name='goal_joint_torques',      data=np.zeros(2))
HEAD_JOINT_COMMAND.add_block(name='bear_enable_statuses',    data=np.zeros(2))
HEAD_JOINT_COMMAND.add_block(name='bear_operating_modes',    data=np.zeros(2))
HEAD_JOINT_COMMAND.add_block(name='bear_proportional_gains', data=np.zeros(2))
HEAD_JOINT_COMMAND.add_block(name='bear_derivative_gains',   data=np.zeros(2))


# Right Hand Joint State
RIGHT_HAND_JOINT_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='RIGHT_HAND_JOINT_STATE', init=False)
RIGHT_HAND_JOINT_STATE.add_block(name='timestamp', data=np.zeros(1))

RIGHT_HAND_JOINT_STATE.add_block(name='joint_positions',  data=np.zeros(7))
RIGHT_HAND_JOINT_STATE.add_block(name='joint_velocities', data=np.zeros(7))
RIGHT_HAND_JOINT_STATE.add_block(name='joint_torques',    data=np.zeros(7))
RIGHT_HAND_JOINT_STATE.add_block(name='dxl_temperatures', data=np.zeros(7))
RIGHT_HAND_JOINT_STATE.add_block(name='dxl_voltages',     data=np.zeros(7))

# Right Hand Command
RIGHT_HAND_JOINT_COMMAND = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='RIGHT_HAND_JOINT_COMMAND', init=False)
RIGHT_HAND_JOINT_COMMAND.add_block(name='timestamp', data=np.zeros(1))

RIGHT_HAND_JOINT_COMMAND.add_block(name='goal_joint_positions',   data=np.zeros(7))
RIGHT_HAND_JOINT_COMMAND.add_block(name='goal_joint_velocities',  data=np.zeros(7))
RIGHT_HAND_JOINT_COMMAND.add_block(name='goal_joint_torques',     data=np.zeros(7))
RIGHT_HAND_JOINT_COMMAND.add_block(name='dxl_enable_statuses',    data=np.zeros(7))
RIGHT_HAND_JOINT_COMMAND.add_block(name='dxl_operating_modes',    data=np.array([3]*7))
RIGHT_HAND_JOINT_COMMAND.add_block(name='dxl_proportional_gains', data=np.zeros(7))
RIGHT_HAND_JOINT_COMMAND.add_block(name='dxl_derivative_gains',   data=np.zeros(7))


# Left Hand Joint State
LEFT_HAND_JOINT_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='LEFT_HAND_JOINT_STATE', init=False)
LEFT_HAND_JOINT_STATE.add_block(name='timestamp', data=np.zeros(1))

LEFT_HAND_JOINT_STATE.add_block(name='joint_positions',  data=np.zeros(7))
LEFT_HAND_JOINT_STATE.add_block(name='joint_velocities', data=np.zeros(7))
LEFT_HAND_JOINT_STATE.add_block(name='joint_torques',    data=np.zeros(7))
LEFT_HAND_JOINT_STATE.add_block(name='dxl_temperatures', data=np.zeros(7))
LEFT_HAND_JOINT_STATE.add_block(name='dxl_voltages',     data=np.zeros(7))

# Left Hand Joint Command
LEFT_HAND_JOINT_COMMAND = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='LEFT_HAND_JOINT_COMMAND', init=False)
LEFT_HAND_JOINT_COMMAND.add_block(name='timestamp', data=np.zeros(1))

LEFT_HAND_JOINT_COMMAND.add_block(name='goal_joint_positions',   data=np.zeros(7))
LEFT_HAND_JOINT_COMMAND.add_block(name='goal_joint_velocities',  data=np.zeros(7))
LEFT_HAND_JOINT_COMMAND.add_block(name='goal_joint_torques',     data=np.zeros(7))
LEFT_HAND_JOINT_COMMAND.add_block(name='dxl_enable_statuses',    data=np.zeros(7))
LEFT_HAND_JOINT_COMMAND.add_block(name='dxl_operating_modes',    data=np.array([3]*7))
LEFT_HAND_JOINT_COMMAND.add_block(name='dxl_proportional_gains', data=np.zeros(7))
LEFT_HAND_JOINT_COMMAND.add_block(name='dxl_derivative_gains',   data=np.zeros(7))


# Base State
BASE_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='BASE_STATE', init=False)
BASE_STATE.add_block(name='timestamp', data=np.zeros(1))

BASE_STATE.add_block(name='base_position',        data=np.array([0., 0., 1.2]))
BASE_STATE.add_block(name='base_velocity',        data=np.zeros(3))
BASE_STATE.add_block(name='base_acceleration',    data=np.zeros(3))
BASE_STATE.add_block(name='base_rotation_matrix', data=np.eye(3))
BASE_STATE.add_block(name='base_heading',         data=np.zeros(1))
BASE_STATE.add_block(name='base_angular_rate',    data=np.zeros(3))

# CoM State
COM_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='COM_STATE', init=False)
COM_STATE.add_block(name='timestamp', data=np.zeros(1))

COM_STATE.add_block(name='com_position',         data=np.zeros(3))
COM_STATE.add_block(name='com_velocity',         data=np.zeros(3))
COM_STATE.add_block(name='linear_momentum',      data=np.zeros(3))
COM_STATE.add_block(name='angular_momentum',     data=np.zeros(3))
COM_STATE.add_block(name='angular_momentum_cop', data=np.zeros(2))
COM_STATE.add_block(name='H_matrix',             data=np.zeros((34, 34)))
COM_STATE.add_block(name='CG_vector',            data=np.zeros(34))
COM_STATE.add_block(name='AG_matrix',            data=np.zeros((6, 34)))
COM_STATE.add_block(name='dAGdq_vector',         data=np.zeros(6))

# Right Foot State
RIGHT_FOOT_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='RIGHT_FOOT_STATE', init=False)
RIGHT_FOOT_STATE.add_block(name='timestamp', data=np.zeros(1))

RIGHT_FOOT_STATE.add_block(name='right_foot_rotation_matrix', data=np.eye(3))
RIGHT_FOOT_STATE.add_block(name='right_foot_angular_rate',    data=np.zeros(3))
RIGHT_FOOT_STATE.add_block(name='right_foot_Jw',              data=np.zeros((3, 12)))
RIGHT_FOOT_STATE.add_block(name='right_foot_dJwdq',           data=np.zeros(3))
RIGHT_FOOT_STATE.add_block(name='right_foot_position',        data=np.zeros(3))
RIGHT_FOOT_STATE.add_block(name='right_foot_velocity',        data=np.zeros(3))
RIGHT_FOOT_STATE.add_block(name='right_foot_Jv',              data=np.zeros((3, 12)))
RIGHT_FOOT_STATE.add_block(name='right_foot_dJvdq',           data=np.zeros(3))
RIGHT_FOOT_STATE.add_block(name='right_foot_contact_state',   data=np.zeros(1))
RIGHT_FOOT_STATE.add_block(name='right_foot_contact_force',   data=np.zeros(3))

# Left Foot State
LEFT_FOOT_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='LEFT_FOOT_STATE', init=False)
LEFT_FOOT_STATE.add_block(name='timestamp', data=np.zeros(1))

LEFT_FOOT_STATE.add_block(name='left_foot_rotation_matrix', data=np.eye(3))
LEFT_FOOT_STATE.add_block(name='left_foot_angular_rate',    data=np.zeros(3))
LEFT_FOOT_STATE.add_block(name='left_foot_Jw',              data=np.zeros((3, 12)))
LEFT_FOOT_STATE.add_block(name='left_foot_dJwdq',           data=np.zeros(3))
LEFT_FOOT_STATE.add_block(name='left_foot_position',        data=np.zeros(3))
LEFT_FOOT_STATE.add_block(name='left_foot_velocity',        data=np.zeros(3))
LEFT_FOOT_STATE.add_block(name='left_foot_Jv',              data=np.zeros((3, 12)))
LEFT_FOOT_STATE.add_block(name='left_foot_dJvdq',           data=np.zeros(3))
LEFT_FOOT_STATE.add_block(name='left_foot_contact_state',   data=np.zeros(1))
LEFT_FOOT_STATE.add_block(name='left_foot_contact_force',   data=np.zeros(3))

# Right Ankle State
RIGHT_ANKLE_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='RIGHT_ANKLE_STATE', init=False)
RIGHT_ANKLE_STATE.add_block(name='timestamp', data=np.zeros(1))

RIGHT_ANKLE_STATE.add_block(name='right_ankle_position', data=np.zeros(3))
RIGHT_ANKLE_STATE.add_block(name='right_ankle_velocity', data=np.zeros(3))
RIGHT_ANKLE_STATE.add_block(name='right_ankle_Jv',       data=np.zeros((3, 12)))
RIGHT_ANKLE_STATE.add_block(name='right_ankle_dJvdq',    data=np.zeros(3))

# Left Ankle State
LEFT_ANKLE_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='LEFT_ANKLE_STATE', init=False)
LEFT_ANKLE_STATE.add_block(name='timestamp', data=np.zeros(1))

LEFT_ANKLE_STATE.add_block(name='left_ankle_position', data=np.zeros(3))
LEFT_ANKLE_STATE.add_block(name='left_ankle_velocity', data=np.zeros(3))
LEFT_ANKLE_STATE.add_block(name='left_ankle_Jv',       data=np.zeros((3, 12)))
LEFT_ANKLE_STATE.add_block(name='left_ankle_dJvdq',    data=np.zeros(3))

# Right Hand State
RIGHT_HAND_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='RIGHT_HAND_STATE', init=False)
RIGHT_HAND_STATE.add_block(name='timestamp', data=np.zeros(1))

RIGHT_HAND_STATE.add_block(name='right_hand_rotation_matrix', data=np.eye(3))
RIGHT_HAND_STATE.add_block(name='right_hand_angular_rate',    data=np.zeros(3))
RIGHT_HAND_STATE.add_block(name='right_hand_Jw',              data=np.zeros((3, 13)))
RIGHT_HAND_STATE.add_block(name='right_hand_dJwdq',           data=np.zeros(3))
RIGHT_HAND_STATE.add_block(name='right_hand_position',        data=np.zeros(3))
RIGHT_HAND_STATE.add_block(name='right_hand_velocity',        data=np.zeros(3))
RIGHT_HAND_STATE.add_block(name='right_hand_Jv',              data=np.zeros((3, 13)))
RIGHT_HAND_STATE.add_block(name='right_hand_dJvdq',           data=np.zeros(3))

# Left Hand State
LEFT_HAND_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='LEFT_HAND_STATE', init=False)
LEFT_HAND_STATE.add_block(name='timestamp', data=np.zeros(1))

LEFT_HAND_STATE.add_block(name='left_hand_rotation_matrix', data=np.eye(3))
LEFT_HAND_STATE.add_block(name='left_hand_angular_rate',    data=np.zeros(3))
LEFT_HAND_STATE.add_block(name='left_hand_Jw',              data=np.zeros((3, 13)))
LEFT_HAND_STATE.add_block(name='left_hand_dJwdq',           data=np.zeros(3))
LEFT_HAND_STATE.add_block(name='left_hand_position',        data=np.zeros(3))
LEFT_HAND_STATE.add_block(name='left_hand_velocity',        data=np.zeros(3))
LEFT_HAND_STATE.add_block(name='left_hand_Jv',              data=np.zeros((3, 13)))
LEFT_HAND_STATE.add_block(name='left_hand_dJvdq',           data=np.zeros(3))

# Head State
HEAD_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='HEAD_STATE', init=False)
HEAD_STATE.add_block(name='timestamp', data=np.zeros(1))

HEAD_STATE.add_block(name='head_rotation_matrix', data=np.eye(3))
HEAD_STATE.add_block(name='head_angular_rate',    data=np.zeros(3))
HEAD_STATE.add_block(name='head_Jw',              data=np.zeros((3, 8)))
HEAD_STATE.add_block(name='head_dJwdq',           data=np.zeros(3))
HEAD_STATE.add_block(name='head_position',        data=np.zeros(3))
HEAD_STATE.add_block(name='head_velocity',        data=np.zeros(3))
HEAD_STATE.add_block(name='head_Jv',              data=np.zeros((3, 8)))
HEAD_STATE.add_block(name='head_dJvdq',           data=np.zeros(3))

# Head Camera State
HEAD_CAMERA_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='HEAD_CAMERA_STATE', init=False)
HEAD_CAMERA_STATE.add_block(name='timestamp', data=np.zeros(1))

HEAD_CAMERA_STATE.add_block(name='head_camera_rotation_matrix', data=np.eye(3))
HEAD_CAMERA_STATE.add_block(name='head_camera_angular_rate',    data=np.zeros(3))
HEAD_CAMERA_STATE.add_block(name='head_camera_position',        data=np.zeros(3))
HEAD_CAMERA_STATE.add_block(name='head_camera_velocity',        data=np.zeros(3))


# Locomotion Planner State
LOCOMOTION_PLANNER_STATE = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='LOCOMOTION_PLANNER_STATE', init=False)
LOCOMOTION_PLANNER_STATE.add_block(name='timestamp', data=np.zeros(1))

LOCOMOTION_PLANNER_STATE.add_block(name='locomotion_mode',  data=np.zeros(1))
LOCOMOTION_PLANNER_STATE.add_block(name='locomotion_phase', data=np.zeros(1))

LOCOMOTION_PLANNER_STATE.add_block(name='base_rotation_matrix', data=np.eye(3))
LOCOMOTION_PLANNER_STATE.add_block(name='base_angular_rate',    data=np.zeros(3))
LOCOMOTION_PLANNER_STATE.add_block(name='base_heading',         data=np.zeros(1))
LOCOMOTION_PLANNER_STATE.add_block(name='base_position',        data=np.zeros(3))
LOCOMOTION_PLANNER_STATE.add_block(name='base_velocity',        data=np.zeros(3))
LOCOMOTION_PLANNER_STATE.add_block(name='com_position',         data=np.zeros(3))
LOCOMOTION_PLANNER_STATE.add_block(name='com_velocity',         data=np.zeros(3))

LOCOMOTION_PLANNER_STATE.add_block(name='right_foot_phase',           data=np.zeros(1))
LOCOMOTION_PLANNER_STATE.add_block(name='right_ankle_position',       data=np.zeros(3))
LOCOMOTION_PLANNER_STATE.add_block(name='right_ankle_velocity',       data=np.zeros(3))
LOCOMOTION_PLANNER_STATE.add_block(name='right_ankle_acceleration',   data=np.zeros(3))
LOCOMOTION_PLANNER_STATE.add_block(name='right_foot_rotation_matrix', data=np.eye(3))
LOCOMOTION_PLANNER_STATE.add_block(name='right_foot_angular_rate',    data=np.zeros(3))
LOCOMOTION_PLANNER_STATE.add_block(name='right_ankle_last_position',  data=np.zeros(3))
LOCOMOTION_PLANNER_STATE.add_block(name='right_ankle_goal_position',  data=np.zeros(3))

LOCOMOTION_PLANNER_STATE.add_block(name='left_foot_phase',            data=np.zeros(1))
LOCOMOTION_PLANNER_STATE.add_block(name='left_ankle_position',        data=np.zeros(3))
LOCOMOTION_PLANNER_STATE.add_block(name='left_ankle_velocity',        data=np.zeros(3))
LOCOMOTION_PLANNER_STATE.add_block(name='left_ankle_acceleration',    data=np.zeros(3))
LOCOMOTION_PLANNER_STATE.add_block(name='left_foot_rotation_matrix',  data=np.eye(3))
LOCOMOTION_PLANNER_STATE.add_block(name='left_foot_angular_rate',     data=np.zeros(3))
LOCOMOTION_PLANNER_STATE.add_block(name='left_ankle_last_position',   data=np.zeros(3))
LOCOMOTION_PLANNER_STATE.add_block(name='left_ankle_goal_position',   data=np.zeros(3))


# User Command
USER_COMMAND = shared_memory_manager.SharedMemoryManager(robot_name='THEMIS', seg_name='USER_COMMAND', init=False)
USER_COMMAND.add_block(name='timestamp', data=np.zeros(1))

USER_COMMAND.add_block(name='locomotion_mode',         data=np.zeros(1))
USER_COMMAND.add_block(name='horizontal_velocity',     data=np.zeros(2))
USER_COMMAND.add_block(name='yaw_rate',                data=np.zeros(1))
USER_COMMAND.add_block(name='com_position_change',     data=np.zeros(3))
USER_COMMAND.add_block(name='base_euler_angle_change', data=np.zeros(3))
USER_COMMAND.add_block(name='right_foot_yaw_change',   data=np.zeros(1))
USER_COMMAND.add_block(name='left_foot_yaw_change',    data=np.zeros(1))
USER_COMMAND.add_block(name='cop_clearance_change',    data=np.zeros(2))


MEMORY_SEGMENT_LIST = [THREAD_STATE, THREAD_COMMAND, SIMULATION_STATE, SENSE_STATE, BATTERY_STATE, GAMEPAD_STATE,
                       RIGHT_LEG_JOINT_STATE, RIGHT_LEG_JOINT_COMMAND,
                       LEFT_LEG_JOINT_STATE, LEFT_LEG_JOINT_COMMAND,
                       RIGHT_ARM_JOINT_STATE, RIGHT_ARM_JOINT_COMMAND,
                       LEFT_ARM_JOINT_STATE, LEFT_ARM_JOINT_COMMAND,
                       HEAD_JOINT_STATE, HEAD_JOINT_COMMAND,
                       RIGHT_HAND_JOINT_STATE, RIGHT_HAND_JOINT_COMMAND,
                       LEFT_HAND_JOINT_STATE, LEFT_HAND_JOINT_COMMAND,
                       BASE_STATE, COM_STATE, RIGHT_FOOT_STATE, LEFT_FOOT_STATE, RIGHT_ANKLE_STATE, LEFT_ANKLE_STATE, RIGHT_HAND_STATE, LEFT_HAND_STATE, HEAD_STATE, HEAD_CAMERA_STATE,
                       LOCOMOTION_PLANNER_STATE, USER_COMMAND]


def init():
    """
    Initialize if main
    """
    THREAD_STATE.initialize     = True
    THREAD_COMMAND.initialize   = True
    SIMULATION_STATE.initialize = True
    SENSE_STATE.initialize      = True
    BATTERY_STATE.initialize    = True
    GAMEPAD_STATE.initialize    = True

    RIGHT_LEG_JOINT_STATE.initialize    = True
    RIGHT_LEG_JOINT_COMMAND.initialize  = True
    LEFT_LEG_JOINT_STATE.initialize     = True
    LEFT_LEG_JOINT_COMMAND.initialize   = True
    RIGHT_ARM_JOINT_STATE.initialize    = True
    RIGHT_ARM_JOINT_COMMAND.initialize  = True
    LEFT_ARM_JOINT_STATE.initialize     = True
    LEFT_ARM_JOINT_COMMAND.initialize   = True
    HEAD_JOINT_STATE.initialize         = True
    HEAD_JOINT_COMMAND.initialize       = True
    RIGHT_HAND_JOINT_STATE.initialize   = True
    RIGHT_HAND_JOINT_COMMAND.initialize = True
    LEFT_HAND_JOINT_STATE.initialize    = True
    LEFT_HAND_JOINT_COMMAND.initialize  = True

    BASE_STATE.initialize        = True
    COM_STATE.initialize         = True
    RIGHT_FOOT_STATE.initialize  = True
    LEFT_FOOT_STATE.initialize   = True
    RIGHT_ANKLE_STATE.initialize = True
    LEFT_ANKLE_STATE.initialize  = True
    RIGHT_HAND_STATE.initialize  = True
    LEFT_HAND_STATE.initialize   = True
    HEAD_STATE.initialize        = True
    HEAD_CAMERA_STATE.initialize = True

    LOCOMOTION_PLANNER_STATE.initialize = True
    USER_COMMAND.initialize  = True


def connect():
    """
    Connect and create segment
    """
    THREAD_STATE.connect_segment()
    THREAD_COMMAND.connect_segment()
    SIMULATION_STATE.connect_segment()
    SENSE_STATE.connect_segment()
    BATTERY_STATE.connect_segment()
    GAMEPAD_STATE.connect_segment()

    RIGHT_LEG_JOINT_STATE.connect_segment()
    RIGHT_LEG_JOINT_COMMAND.connect_segment()
    LEFT_LEG_JOINT_STATE.connect_segment()
    LEFT_LEG_JOINT_COMMAND.connect_segment()
    RIGHT_ARM_JOINT_STATE.connect_segment()
    RIGHT_ARM_JOINT_COMMAND.connect_segment()
    LEFT_ARM_JOINT_STATE.connect_segment()
    LEFT_ARM_JOINT_COMMAND.connect_segment()
    HEAD_JOINT_STATE.connect_segment()
    HEAD_JOINT_COMMAND.connect_segment()
    RIGHT_HAND_JOINT_STATE.connect_segment()
    RIGHT_HAND_JOINT_COMMAND.connect_segment()
    LEFT_HAND_JOINT_STATE.connect_segment()
    LEFT_HAND_JOINT_COMMAND.connect_segment()
    
    BASE_STATE.connect_segment()
    COM_STATE.connect_segment()
    RIGHT_FOOT_STATE.connect_segment()
    LEFT_FOOT_STATE.connect_segment()
    RIGHT_ANKLE_STATE.connect_segment()
    LEFT_ANKLE_STATE.connect_segment()
    RIGHT_HAND_STATE.connect_segment()
    LEFT_HAND_STATE.connect_segment()
    HEAD_STATE.connect_segment()
    HEAD_CAMERA_STATE.connect_segment()
    
    LOCOMOTION_PLANNER_STATE.connect_segment()
    USER_COMMAND.connect_segment()


if __name__ == '__main__':
    init()
    connect()
else:
    connect()