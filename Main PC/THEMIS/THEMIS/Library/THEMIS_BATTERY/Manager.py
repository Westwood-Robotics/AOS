#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "March 3, 2025"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Serial communication with battery
'''

import time
import serial
import numpy as np


class BatteryManager(object):
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        # serial communication
        self.port = port
        self.baudrate = baudrate
        self.ser = None

        # data from pico
        self.data4pico = [0xFF, 0xFF, 0x00, 28] + [0x00] * 28 + [0x00]  # [0xFF, 0xFF, ID, Length, Data, Checksum]
        self.data4pico_data_length     = 28
        self.data4pico_packet_length   = len(self.data4pico)
        self.data4pico_packet_length_1 = len(self.data4pico) - 1

        self.battery_status        = np.zeros(2)
        self.error                 = np.zeros(2)
        self.battery_voltage       = np.zeros(2)
        self.cell_voltage          = np.zeros((2, 8))
        self.dsg_status            = np.zeros(2)
        self.chg_status            = np.zeros(2)
        self.max_discharge_current = np.zeros(2)
        self.max_charge_current    = np.zeros(2)
        self.present_current       = np.zeros(2)
        self.temperature           = np.zeros(2)

        # initialize
        self.open_port()

    def open_port(self):
        """
        Open the serial port
        """
        self.ser = serial.Serial(self.port,
                                 self.baudrate,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=0)

    def close_port(self):
        """
        Close the serial port
        """
        if self.ser:
            self.ser.reset_input_buffer()
            self.ser.close()

    def get_data(self):
        """
        Get data from serial port if available
        """
        try:
            while self.ser.in_waiting >= 4:
                if self.ser.read(1)[0] == 0xFF and self.ser.read(1)[0] == 0xFF:  # find first and second bytes
                    self.data4pico[2] = self.ser.read(1)[0]
                    self.data4pico[3] = self.ser.read(1)[0]
                    if self.data4pico[3] == 0:
                        for idx in range(4, self.data4pico_packet_length_1):
                            self.data4pico[idx] = None
                        self.data4pico[-1] = self.ser.read(1)[0]
                    elif self.data4pico[3] == self.data4pico_data_length:
                        for idx in range(4, self.data4pico_packet_length):
                            self.data4pico[idx] = self.ser.read(1)[0]

                    # print(self.data4pico, self.data4pico[2], time.time())
                    if self.checksum(self.data4pico) == self.data4pico[-1]:
                        id = self.data4pico[2]

                        if self.data4pico[3] == 0:
                            self.battery_status[id]        = 0
                            self.error[id]                 = -329
                            self.battery_voltage[id]       = -329
                            self.cell_voltage[id, :]       = np.array([-329] * 8)
                            self.dsg_status[id]            = -329
                            self.chg_status[id]            = -329
                            self.max_discharge_current[id] = -329
                            self.max_charge_current[id]    = -329
                            self.present_current[id]       = -329
                            self.temperature[id]           = -329
                        else:
                            self.battery_status[id]        = 1
                            self.error[id]                 =   self.data4pico[5]
                            self.battery_voltage[id]       =  (self.data4pico[6]  << 8 | self.data4pico[7])  / 1000
                            self.cell_voltage[id, :]       = [(self.data4pico[8]  << 8 | self.data4pico[9])  / 1000,
                                                              (self.data4pico[10] << 8 | self.data4pico[11]) / 1000,
                                                              (self.data4pico[12] << 8 | self.data4pico[13]) / 1000,
                                                              (self.data4pico[14] << 8 | self.data4pico[15]) / 1000,
                                                              (self.data4pico[16] << 8 | self.data4pico[17]) / 1000,
                                                              (self.data4pico[18] << 8 | self.data4pico[19]) / 1000,
                                                              (self.data4pico[20] << 8 | self.data4pico[21]) / 1000,
                                                              (self.data4pico[22] << 8 | self.data4pico[23]) / 1000]
                            self.dsg_status[id]            =  (self.data4pico[24] & 0b00000010) / 2
                            self.chg_status[id]            =   self.data4pico[24] & 0b00000001
                            self.max_discharge_current[id] =  (self.data4pico[25] << 8 | self.data4pico[26]) / 1000
                            self.max_charge_current[id]    =  (self.data4pico[27] << 8 | self.data4pico[28]) / 1000
                            self.present_current[id]       = int.from_bytes(bytes(self.data4pico[29:31]), byteorder='big', signed=True) / 1000
                            self.temperature[id]           = self.data4pico[31]
                        
                        return True
                                        
            return False
        except:
            return False

    @staticmethod
    def checksum(packet):
        if packet[3] == 0:
            return packet[2]
        else:
            return sum(packet[2:-1]) % 256