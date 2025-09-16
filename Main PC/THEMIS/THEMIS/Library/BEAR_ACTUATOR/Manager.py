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
BEAR Actuator Manager
'''

from termcolor import cprint
from Library.BEAR_ACTUATOR import Packet
from Library.BEAR_ACTUATOR.CONTROL_TABLE import *


class BearActuatorManager(Packet.PKT):
    def __init__(self, port='/dev/ttyUSB0', baudrate=8000000):
        # serial communication
        self.baudrate = baudrate
        self.port = port
        super(BearActuatorManager, self).__init__(self.port, self.baudrate)

        # BEAR parameters
        self.enable_status  = {'disable': 0, 'enable': 1, 'error': 2, 'damping': 3}
        self.operating_mode = {'torque': 0, 'velocity': 1, 'position': 2, 'force': 3}
        self.error_status   = {'Communication': 0, 'Overheat': 1, 'Absolute Position': 2, 'E-STOP': 3, 'Joint Limit': 4, 'Hardware Fault': 5, 'Initialization': 6}

    # Error
    def decode_error(self, error_code):
        """
        Decode BEAR error code, e.g., decode_error(code)
        return: error names
        """
        msg = ''
        if error_code >> 7 != 1:
            cprint("Invalid Error Code!!!", 'red')
            return msg
        else:
            error_num = 0
            for idx in range(7):
                if error_code >> idx & 1:
                    error_num += 1
                    if error_num > 1:
                        msg += ' & '
                    # msg += self.error_status[idx]
                    msg += list(self.error_status.keys())[list(self.error_status.values()).index(idx)]
            if error_num:
                return msg
            else:
                return "No Error!"

    # Ping
    def ping(self, bear_list):
        """
        Ping BEARs, e.g., ping([id1, id2])
        return: list of pinging results (None if undetected)
        """
        return [self._ping(bear) for bear in bear_list]

    # Save Config
    def save_config(self, bear_list):
        """
        Save BEAR configuration registers, e.g., save_config([id1, id2])
        """
        for bear in bear_list:
            self._save_config(bear)

    # Set Absolute Position
    def set_posi(self, bear_list, position_list, tolerance_list):
        """
        Set BEAR absolute position, e.g., set_posi([id1, id2], [pos1, pos2], [tol1, tol2])
        """
        for idx, bear in enumerate(bear_list):
            self._set_posi(bear, position_list[idx], tolerance_list[idx])

    # Single Read/Write
    def single_read(self, bear, reg_name_list):
        """
        Read multiple registers of a single BEAR, e.g., single_read(id, [name1, name2])
        return: list of register data, e.g., [data1, data2]
        """
        cfg_name_list = []
        stat_name_list = []
        for idx, reg_name in enumerate(reg_name_list):
            if REGISTER['REG_TYPE'][reg_name] == 'cfg':
                cfg_name_list.append(reg_name)
            elif REGISTER['REG_TYPE'][reg_name] == 'stat':
                stat_name_list.append(reg_name)

        if cfg_name_list:
            cfg_data, cfg_error = self._single_read(bear, cfg_name_list, reg_type='cfg')

        if stat_name_list:
            stat_data, stat_error = self._single_read(bear, stat_name_list, reg_type='stat')

        reg_data = []
        for reg_name in reg_name_list:
            if REGISTER['REG_TYPE'][reg_name] == 'cfg':
                reg_data.append(cfg_data[0])
                cfg_data.pop(0)
            elif REGISTER['REG_TYPE'][reg_name] == 'stat':
                reg_data.append(stat_data[0])
                stat_data.pop(0)
        return reg_data

    def single_write(self, bear, reg_name_list, data_list):
        """
        Write multiple registers of a single BEAR, e.g., single_write(id, [name1, name2], [data1, data2])
        """
        cfg_name_list = []
        cfg_data_list = []
        stat_name_list = []
        stat_data_list = []
        for idx, reg_name in enumerate(reg_name_list):
            data = data_list[idx]
            if REGISTER['REG_TYPE'][reg_name] == 'cfg':
                cfg_name_list.append(reg_name)
                cfg_data_list.append(data)
            elif REGISTER['REG_TYPE'][reg_name] == 'stat':
                stat_name_list.append(reg_name)
                stat_data_list.append(data)

        if cfg_name_list:
            self._single_write(bear, cfg_name_list, cfg_data_list, reg_type='cfg')

        if stat_data_list:
            self._single_write(bear, stat_name_list, stat_data_list, reg_type='stat')

    def bulk_read_stat(self, bear_list, read_stat_name_list):
        """
        Read multiple status registers of multiple BEARs, e.g., bulk_read_stat([id1, id2], [read_name1, read_name2])
        return: list of status data, e.g., [[read_data11, read_data12], [read_data21, read_data22]]
        """
        read_stat_data_list, error_list = self._bulk_read_write_stat(bear_list, read_stat_name_list, [], [])
        return read_stat_data_list

    def bulk_write_stat(self, bear_list, write_stat_name_list, data_list):
        """
        Write multiple status registers of multiple BEARs, e.g., bulk_write_stat([id1, id2], [write_name1, write_name2], [[write_data11, write_data12], [write_data21, write_data22]])
        """
        self._bulk_read_write_stat(bear_list, [], write_stat_name_list, data_list)

    def bulk_read_write_stat(self, bear_list, read_stat_name_list, write_stat_name_list, write_stat_data_list):
        """
        Read and write multiple status registers of multiple BEARs, e.g., bulk_read_write_stat([id1, id2], [read_name1, read_name2], [write_name1, write_name2], [[write_data11, write_data12], [write_data21, write_data22]])
        return: list of status data, e.g., [[read_data11, read_data12], [read_data21, read_data22]]
        """
        read_stat_data_list, error_list = self._bulk_read_write_stat(bear_list, read_stat_name_list, write_stat_name_list, write_stat_data_list)
        return read_stat_data_list

    # ID
    def set_ID(self, bear_list, id_list):
        """
        Set BEAR ID, e.g., set_ID([id1, id2], [new_id1, new_id2])
        """
        if len(bear_list) == len(id_list):
            for idx, bear in enumerate(bear_list):
                self.single_write(bear, ['id'], [id_list[idx]])
        else:
            cprint("ID Mismatch!!!", 'red')

    # Enable Status
    def get_enable_status(self, bear_list):
        """
        Get BEAR enable status, e.g., get_enable_status([id1, id2])
        return: list of enable status info, e.g., [status1, status2]
        status: 0 - disable
                1 - enable
                2 - error
                3 - damping
        """
        if len(bear_list) > 1:
            enable_status_info = self.bulk_read_stat(bear_list, ['torque_enable'])
            return [val[0] for val in enable_status_info]
        else:
            return self.single_read(bear_list[0], ['torque_enable'])

    def is_enable_status(self, bear_list, target):
        """
        Is BEARs in target enable status, e.g., is_enable_status([id1, id2], status)
        return: list of bools, e.g., [bool1, bool2]
        """
        enable_status_info = self.get_enable_status(bear_list)
        if target in ['disable', 'enable', 'error', 'damping']:
            return [enable_status_info[idx] == self.enable_status[target] if enable_status_info[idx] is not None else None for idx in range(len(bear_list))]
        else:
            cprint("Invalid Enable Status!!!", 'red')
            return [None] * len(bear_list)

    def set_enable_status(self, bear_list, target):
        """
        Set BEARs into target enable status, e.g., set_enable_status([id1, id2], status)
        """
        if target in ['disable', 'enable', 'damping']:
            self.bulk_write_stat(bear_list, ['torque_enable'], [[self.enable_status[target]]] * len(bear_list))
        else:
            cprint("Invalid Enable Status!!!", 'red')

    # Operating Mode
    def get_operating_mode(self, bear_list):
        """
        Get BEAR operating mode, e.g., get_operating_mode([id1, id2])
        return: list of operating mode info, e.g., [mode1, mode2]
        mode: 0 - torque
              1 - velocity
              2 - position
              3 - force
        """
        return [self.single_read(bear, ['mode'])[0] for bear in bear_list]

    def is_operating_mode(self, bear_list, target):
        """
        Is BEARs in target operating mode, e.g., is_operating_mode([id1, id2], mode)
        return: list of bools, e.g., [bool1, bool2]
        """
        operating_mode_info = self.get_operating_mode(bear_list)
        if target in ['torque', 'velocity', 'position', 'force']:
            return [operating_mode_info[idx] == self.operating_mode[target] if operating_mode_info[idx] is not None else None for idx in range(len(bear_list))]
        else:
            cprint("Invalid Operating Mode!!!", 'red')
            return [None] * len(bear_list)

    def set_operating_mode(self, bear_list, target):
        """
        Set BEARs into target operating mode, e.g., set_operating_mode([id1, id2], mode)
        """
        if target in ['torque', 'velocity', 'position', 'force']:
            for bear in bear_list:
                self.single_write(bear, ['mode'], [self.operating_mode[target]])
        else:
            cprint("Invalid Operating Mode!!!", 'red')

    # PID Gain
    def get_pid(self, bear_list, target):
        """
        Get BEAR pid gain, e.g., get_pid_gain([id1, id2], target)
        return: list of pid gain info, e.g., [[kp1, ki1, kd1], [kp2, ki2, kd2]]
        """
        if target in ['velocity', 'position', 'force', 'iq', 'id']:
            p_gain_info = [self.single_read(bear, ['p_gain_' + target])[0] for bear in bear_list]
            i_gain_info = [self.single_read(bear, ['i_gain_' + target])[0] for bear in bear_list]
            d_gain_info = [self.single_read(bear, ['d_gain_' + target])[0] for bear in bear_list]
            return [[p_gain_info[idx], i_gain_info[idx], d_gain_info[idx]] for idx in range(len(bear_list))]
        else:
            cprint("Invalid PID Target!!!", 'red')
            return [[None, None, None] * len(bear_list)]

    def set_pid(self, bear_list, target, pid_list):
        """
        Set BEAR pid gain, e.g., set_pid_gains([id1, id2], target, [[kp, ki, kd], [kp, ki, kd]])
        """
        if len(bear_list) == len(pid_list):
            if target in ['velocity', 'position', 'force', 'iq', 'id']:
                for idx, bear in enumerate(bear_list):
                    if len(pid_list[idx]) == 3:
                        self.single_write(bear, ['p_gain_' + target, 'i_gain_' + target, 'd_gain_' + target], [pid_list[idx][0], pid_list[idx][1], pid_list[idx][2]])
                    else:
                        cprint("PID Mismatch!!!", 'red')
            else:
                cprint("Invalid PID Target!!!", 'red')
        else:
            cprint("PID Mismatch!!!", 'red')

    # Limit
    def get_limit(self, bear_list, target):
        """
        Get BEAR limit, e.g., get_limit([id1, id2], target)
        return: list of limit info, e.g., [[min1, max1], [min2, max2]]
        """
        if target == 'velocity':
            max_info = [self.single_read(bear, ['limit_velocity_max'])[0] for bear in bear_list]
            return [[-max_info[idx], max_info[idx]] for idx in range(len(bear_list))]
        elif target == 'position':
            min_info = [self.single_read(bear, ['limit_position_min'])[0] for bear in bear_list]
            max_info = [self.single_read(bear, ['limit_position_max'])[0] for bear in bear_list]
            return [[min_info[idx], max_info[idx]] for idx in range(len(bear_list))]
        elif target == 'i':
            max_info = [self.single_read(bear, ['limit_i_max'])[0] for bear in bear_list]
            return [[-max_info[idx], max_info[idx]] for idx in range(len(bear_list))]
        elif target == 'voltage':
            min_info = [self.single_read(bear, ['min_voltage'])[0] for bear in bear_list]
            max_info = [self.single_read(bear, ['max_voltage'])[0] for bear in bear_list]
            return [[min_info[idx], max_info[idx]] for idx in range(len(bear_list))]
        elif target == 'temperature':
            min_info = [self.single_read(bear, ['temp_limit_low'])[0] for bear in bear_list]
            max_info = [self.single_read(bear, ['temp_limit_high'])[0] for bear in bear_list]
            return [[min_info[idx], max_info[idx]] for idx in range(len(bear_list))]
        else:
            cprint("Invalid Limit Target!!!", 'red')
            return [[None, None] * len(bear_list)]

    def set_limit(self, bear_list, target, limit_list):
        """
        Set BEAR limit, e.g., set_pid_gains([id1, id2], target, [[min1, max1], [min2, max2]])
        """
        if len(bear_list) == len(limit_list):
            if target == 'velocity':
                for idx, bear in enumerate(bear_list):
                    if len(limit_list[idx]) == 2:
                        self.single_write(bear, ['limit_velocity_max'], [limit_list[idx][1]])
                    else:
                        cprint("Limit Mismatch!!!", 'red')
            elif target == 'position':
                for idx, bear in enumerate(bear_list):
                    if len(limit_list[idx]) == 2:
                        self.single_write(bear, ['limit_position_min', 'limit_position_max'], [limit_list[idx][0], limit_list[idx][1]])
                    else:
                        cprint("Limit Mismatch!!!", 'red')
            elif target == 'i':
                for idx, bear in enumerate(bear_list):
                    if len(limit_list[idx]) == 2:
                        self.single_write(bear, ['limit_i_max'], [limit_list[idx][1]])
                    else:
                        cprint("Limit Mismatch!!!", 'red')
            elif target == 'voltage':
                for idx, bear in enumerate(bear_list):
                    if len(limit_list[idx]) == 2:
                        self.single_write(bear, ['min_voltage', 'max_voltage'], [limit_list[idx][0], limit_list[idx][1]])
                    else:
                        cprint("Limit Mismatch!!!", 'red')
            elif target == 'temperature':
                for idx, bear in enumerate(bear_list):
                    if len(limit_list[idx]) == 2:
                        self.single_write(bear, ['temp_limit_low', 'temp_limit_high'], [limit_list[idx][0], limit_list[idx][1]])
                    else:
                        cprint("Limit Mismatch!!!", 'red')
            else:
                cprint("Invalid Limit Target!!!", 'red')
        else:
            cprint("Limit Mismatch!!!", 'red')

    # Temperature
    def get_temperature(self, bear_list, target):
        """
        Get BEAR temperature, e.g., get_temperature([id1, id2], target)
        return: list of temperature info, e.g., [data1, data2]
        """
        if target in ['winding', 'powerstage', 'ic']:
            temperature_info = self.bulk_read_stat(bear_list, [target + '_temperature'])
            return [val[0] for val in temperature_info]
        else:
            cprint("Invalid Temperature Target!!!", 'red')

    # Voltage
    def get_voltage(self, bear_list):
        """
        Get BEAR voltage, e.g., get_voltage([id1, id2])
        return: list of voltage info, e.g., [data1, data2]
        """
        voltage_info = self.bulk_read_stat(bear_list, ['input_voltage'])
        return [val[0] for val in voltage_info]

    # Position
    def get_present_position(self, bear_list):
        """
        Get BEAR present position, e.g., get_present_position([id1, id2])
        return: list of position info, e.g., [data1, data2]
        """
        position_info = self.bulk_read_stat(bear_list, ['present_position'])
        return [val[0] for val in position_info]
    
    # Velocity
    def get_present_velocity(self, bear_list):
        """
        Get BEAR present velocity, e.g., get_present_velocity([id1, id2])
        return: list of velocity info, e.g., [data1, data2]
        """
        velocity_info = self.bulk_read_stat(bear_list, ['present_velocity'])
        return [val[0] for val in velocity_info]
    
    # Current
    def get_present_current(self, bear_list):
        """
        Get BEAR present current, e.g., get_present_current([id1, id2])
        return: list of current info, e.g., [data1, data2]
        """
        current_info = self.bulk_read_stat(bear_list, ['present_iq'])
        return [val[0] for val in current_info]
    
    # Homing Offset
    def get_homing_offset(self, bear_list):
        """
        Get BEAR homing offset, e.g., get_homing_offset([id1, id2])
        return: list of offset info, e.g., [data1, data2]
        """
        return [self.single_read(bear, ['homing_offset'])[0] for bear in bear_list]
    
    def set_homing_offset(self, bear_list, offset_list):
        """
        Set BEAR homing offset, e.g., set_homing_offset([id1, id2], [offset1, offset2])
        """
        if len(bear_list) == len(offset_list):
            for idx, bear in enumerate(bear_list):
                self.single_write(bear, ['homing_offset'], [offset_list[idx]])
        else:
            cprint("Offset Mismatch!!!", 'red')