#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "February 21, 2025"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
DXL Actuator Manager
'''

from termcolor import cprint
from Setting.Macros.dxl_macros import *
from Library.DXL_ACTUATOR.dynamixel_sdk import *


class DXLActuatorManager():
    def __init__(self, port='/dev/ttyUSB0', baudrate=2000000):
        # serial communication
        self.baudrate = baudrate
        self.port = port

        self.PROTOCOL_VERSION = 2.0

        self.port_handler = None
        self.packet_handler = None

        self.open_port()

        self.sync_write_number = 0
        self.sync_write_blocks = []

        self.sync_read_number = 0
        self.sync_read_blocks = []

        # DXL parameters
        self.enable_status  = {'disable': 0, 'enable': 1}
        self.operating_mode = {'torque': 0, 'velocity': 1, 'position': 3}
        self.error_status   = {'Input Voltage': 0, 'Overheat': 2, 'Electrical Shock': 4, 'Overload': 5}

    def open_port(self):
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        self.port_handler.openPort()
        self.port_handler.setBaudRate(self.baudrate)
        
    def decode_error(self, error_code):
        """
        Decode DXL hardware error code, e.g., decode_error(code)
        return: error names
        """
        msg = ''
        if error_code >> 7 != 1:
            cprint("Invalid Error Code!!!", 'red')
            return msg
        else:
            error_num = 0
            for idx in list(self.error_status.values()):
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
            
    def reboot(self, dxl_list):
        """
        Reboot DXLs, e.g., reboot([id1, id2])
        """
        for dxl in dxl_list:
            comm_result, dxl_error = self.packet_handler.reboot(self.port_handler, dxl)

    def ping(self, dxl_list):
        """
        Ping DXLs, e.g., ping([id1, id2])
        Return: list of pinging results
        """
        return [self.packet_handler.ping(self.port_handler, dxl) for dxl in dxl_list]
    
    def read(self, dxl_list, register_list):
        """
        Read multiple registers of multiple DXLs one by one, e.g., read([id1, id2], [[add1, len1], [add2, len2], [add3, len3]])
        Return: list of data and list of error, e.g., [[data11, data12, data13], [data21, data22, data23]], [err1, err2]
        """
        address_list = [register[0] for register in register_list]
        length_list  = [register[1] for register in register_list]

        data_list = []
        error_list = []
        for dxl in dxl_list:
            data = []
            for idx, address in enumerate(address_list):
                if length_list[idx] == 4:
                    dxl_data, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, dxl, address)
                elif length_list[idx] == 2:
                    dxl_data, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, dxl, address)
                else:
                    dxl_data, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, dxl, address)

                if dxl_comm_result == COMM_SUCCESS:
                    data.append(dxl_data)
                    error = dxl_error
                else:
                    data.append(None)
                    error = None
            data_list += [data]
            error_list.append(error)

        return data_list, error_list
    
    def write(self, dxl_list, register_list, data_list):
        """
        Write multiple registers of multiple DXLs one by one, e.g., write([id1, id2], [[add1, len1], [add2, len2], [add3, len3]], [[data11, data12, data13], [data21, data22, data23]])
        Return: list of error, e.g., [err1, err2]
        """
        address_list = [register[0] for register in register_list]
        length_list  = [register[1] for register in register_list]

        error_list = []
        for idx, dxl in enumerate(dxl_list):
            for jdx, address in enumerate(address_list):
                if length_list[jdx] == 4:
                    dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl, address, data_list[idx][jdx])
                elif length_list[jdx] == 2:
                    dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, dxl, address, data_list[idx][jdx])
                else:
                    dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl, address, data_list[idx][jdx])

            if dxl_comm_result == COMM_SUCCESS:
                error = dxl_error
            else:
                error = None
            error_list.append(error)
        
        return error_list
    
    def sync_write_setup(self, dxl_list, register_list, start_indirect_id=None):
        """
        Initialize GroupSyncWrite instance, e.g., sync_write_setup([id1, id2], [[add1, len1], [add2, len2], [add3, len3]], id)
        Return: GroupSyncWrite block
        """
        address_list = [register[0] for register in register_list]
        length_list  = [register[1] for register in register_list]

        if start_indirect_id is None:
            indirect_address = None
            group_sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, address_list[0], sum(length_list))
        else:
            indirect_address = [DXL_REGISTER_INDIRECT_ADDRESS[0] + start_indirect_id * 2, DXL_REGISTER_INDIRECT_DATA[0] + start_indirect_id]
            group_sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, indirect_address[1], sum(length_list))
            for dxl in dxl_list:
                idx = -2
                for jdx, address in enumerate(address_list):
                    for kdx in range(length_list[jdx]):
                        idx += 2
                        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, dxl, indirect_address[0] + idx, address + kdx)
                        if dxl_comm_result != COMM_SUCCESS:
                            cprint("[DXL ID: %02d] " % dxl + "%s" % self.packet_handler.getTxRxResult(dxl_comm_result), 'yellow')
                        elif dxl_error != 0:
                            cprint("[DXL ID: %02d] " % dxl + "%s" % self.packet_handler.getRxPacketError(dxl_error), 'yellow')

        block = {'id':               self.sync_write_number,
                 'dxl_list':         dxl_list,
                 'address_list':     address_list,
                 'length_list':      length_list,
                 'indirect_address': indirect_address,
                 'instance':         group_sync_write}
        self.sync_write_blocks.append(block)
        self.sync_write_number += 1

        return block

    def sync_read_setup(self, dxl_list, register_list, start_indirect_id=None):
        """
        Initialize GroupSyncRead instance, e.g., sync_read_setup([id1, id2], [[add1, len1], [add2, len2], [add3, len3]], id)
        Return: GroupSyncRead block
        """
        address_list = [register[0] for register in register_list]
        length_list  = [register[1] for register in register_list]

        if start_indirect_id is None:
            indirect_address = None
            group_sync_read = GroupSyncRead(self.port_handler, self.packet_handler, address_list[0], sum(length_list))
            for dxl in dxl_list:
                dxl_addparam_result = group_sync_read.addParam(dxl)
                if dxl_addparam_result is not True:
                    cprint("[DXL ID: %02d] groupSyncRead addparam failed" % dxl, 'yellow')
        else:
            indirect_address = [DXL_REGISTER_INDIRECT_ADDRESS[0] + start_indirect_id * 2, DXL_REGISTER_INDIRECT_DATA[0] + start_indirect_id]
            group_sync_read = GroupSyncRead(self.port_handler, self.packet_handler, indirect_address[1], sum(length_list))
            for dxl in dxl_list:
                idx = -2
                for jdx, address in enumerate(address_list):
                    for kdx in range(length_list[jdx]):
                        idx += 2
                        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, dxl, indirect_address[0] + idx, address + kdx)
                        if dxl_comm_result != COMM_SUCCESS:
                            cprint("[DXL ID: %02d] " % dxl + "%s" % self.packet_handler.getTxRxResult(dxl_comm_result), 'yellow')
                        elif dxl_error != 0:
                            cprint("[DXL ID: %02d] " % dxl + "%s" % self.packet_handler.getRxPacketError(dxl_error), 'yellow')

                dxl_addparam_result = group_sync_read.addParam(dxl)
                if dxl_addparam_result is not True:
                    cprint("[DXL ID: %02d] groupSyncRead addparam failed" % dxl, 'yellow')
                    # quit()

        block = {'id':               self.sync_read_number,
                 'dxl_list':         dxl_list,
                 'address_list':     address_list,
                 'length_list':      length_list,
                 'aug_length_list':  [0] + length_list,
                 'data_list':        [[None] * len(register_list) for _ in range(len(dxl_list))],
                 'indirect_address': indirect_address,
                 'instance':         group_sync_read}
        self.sync_read_blocks.append(block)
        self.sync_read_number += 1

        return block

    def sync_write(self, sync_write_block, data_list):
        """
        GroupSyncWrite data, sync_write(sync_write_block, [id1, id2], [[data11, data12, data13], [data21, data22, data23]])
        """
        dxl_list         = sync_write_block['dxl_list']
        length_list      = sync_write_block['length_list']
        group_sync_write = sync_write_block['instance']

        group_sync_write.clearParam()
        for idx, dxl in enumerate(dxl_list):
            val = []
            for jdx, data in enumerate(data_list[idx]):
                if length_list[jdx] == 4:
                    val += [DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data))]
                elif length_list[jdx] == 2:
                    val += [DXL_LOBYTE(data), DXL_HIBYTE(data)]
                else:
                    val += [data]

            dxl_addparam_result = group_sync_write.addParam(dxl, val)
            if dxl_addparam_result is not True:
                cprint("[DXL ID: %02d] groupSyncWrite addparam failed" % dxl, 'yellow')

        dxl_comm_result = group_sync_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            cprint("[SYNC WRITE ID: %02d] " % sync_write_block['id'] + "%s" % self.packet_handler.getTxRxResult(dxl_comm_result), 'yellow')

    def sync_read(self, sync_read_block):
        """
        GroupSyncRead data, sync_read(sync_read_block)
        Return: list of data, e.g., [[data11, data12, data13], [data21, data22, data23]]
        """
        data_list       = sync_read_block['data_list'].copy()
        group_sync_read = sync_read_block['instance']
        dxl_comm_result = group_sync_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            cprint("[SYNC READ ID: %02d] " % sync_read_block['id'] + "%s" % self.packet_handler.getTxRxResult(dxl_comm_result), 'yellow')
            return None
        else:
            dxl_list        = sync_read_block['dxl_list']
            length_list     = sync_read_block['length_list']
            aug_length_list = sync_read_block['aug_length_list']
            start_address   = sync_read_block['address_list'][0] if sync_read_block['indirect_address'] is None else sync_read_block['indirect_address'][1]
            group_sync_read = sync_read_block['instance']
            for idx, dxl in enumerate(dxl_list):
                address = start_address
                for jdx, length in enumerate(length_list):
                    address += aug_length_list[jdx]
                    data_list[idx][jdx] = group_sync_read.getData(dxl, address, length)
                    # if group_sync_read.isAvailable(dxl, address, length):
                    #     data_list[idx][jdx] = group_sync_read.getData(dxl, address, length)
                    # else:
                    #     data_list[idx][jdx] = None
                    #     print("[DXL ID:%02d] groupSyncRead getdata failed" % DXL_ID)
        
            return data_list