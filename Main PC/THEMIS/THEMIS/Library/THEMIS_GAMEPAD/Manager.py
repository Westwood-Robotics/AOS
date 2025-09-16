#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "May 5, 2025"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Communication with Steam Deck gamepad based on LCM
'''

import lcm
import time
from termcolor import colored
from Library.THEMIS_GAMEPAD.GamepadData import GamepadData


class GamepadManager(object):
    def __init__(self):
        # self.lcm = lcm.LCM()
        self.lcm = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.lcm_channel = "GAMEPAD_DATA"

        self.lcm.subscribe(self.lcm_channel, self.lcm_callback)

        # gamepad status
        self.button = {'U':   0,  # HAT_UP
                       'D':   0,  # HAT_DOWN
                       'L':   0,  # HAT_LEFT
                       'R':   0,  # HAT_RIGHT
                       'A':   0,
                       'B':   0,
                       'X':   0,
                       'Y':   0,

                       'LZ':  0,  # LEFT_AXIS_Z
                       'LS':  0,  # LEFT_SHOULDER
                       'LSP': 0,  # LEFT_SHOULDER_PLUS
                       'LSM': 0,  # LEFT_SHOULDER_MINUS
                       'LS2': 0,  # LEFT_SHOULDER_2
                       'RZ':  0,  # RIGHT_AXIS_Z
                       'RS':  0,  # RIGHT_SHOULDER
                       'RSP': 0,  # RIGHT_SHOULDER_PLUS
                       'RSM': 0,  # RIGHT_SHOULDER_MINUS
                       'RS2': 0,  # RIGHT_SHOULDER_2

                       'ST':  0,  # START
                       'BK':  0,  # BACK
                       'ALT': 0,
                       'FN':  0,
                       }
        self.axis = {'LX':  0,  # LEFT_AXIS_X
                     'LY':  0,  # LEFT_AXIS_Y
                     'RX':  0,  # RIGHT_AXIS_X
                     'RY':  0,  # RIGHT_AXIS_Y
                     'LSZ': 0,  # LEFT_SHOULDER_AXIS_Z
                     'RSZ': 0   # RIGHT_SHOULDER_AXIS_Z
                     }
        
        self.is_connected = False
        self.error_count = 0

        self.print_message(message='connecting')
        self.connect()
        
    def connect(self):
        t0 = time.time()
        while True:
            if (time.time() - t0 > 5):
                self.print_message(message='connection_timeout')
            if self.read_data() and time.time() - t0 > 0.1:
                self.is_connected = True
                self.print_message(message='connected')
                break
            time.sleep(0.1)

    def disconnect(self):
        pass

    def reconnect(self):
        self.print_message(message='reconnecting')
        self.connect()

    def check_connection(self):
        if self.error_count > 100:
            self.is_connected = False
            self.print_message(message='connection_lost')
            for key in list(self.button.keys()):
                self.button[key] = 0.
            for key in list(self.axis.keys()):
                self.axis[key] = 0.

    # def send_data(self, BOOT=0, BEAR=0, INITIALIZE=0, ESTIMATION=0, LOW_LEVEL=0, HIGH_LEVEL=0, TOP_LEVEL=0, LOCOMOTION_MODE=0, BEAR_TEMPERATURE=20, BEAR_VOLTAGE=30):
    #     """
    #     Send data to gamepad via LCM
    #     """
    #     pass
        
    def lcm_callback(self, channel, data):
        """
        Callback function for LCM subscription
        """
        try:
            msg = GamepadData.decode(data)

            self.button['U']  = msg.U
            self.button['D']  = msg.D
            self.button['L']  = msg.L
            self.button['R']  = msg.R
            self.button['A']  = msg.A
            self.button['B']  = msg.B
            self.button['X']  = msg.X
            self.button['Y']  = msg.Y
            self.button['LS'] = msg.LS
            self.button['RS'] = msg.RS
            self.button['BK'] = msg.BK
            self.button['ST'] = msg.ST
            self.button['LZ'] = msg.LZ
            self.button['RZ'] = msg.RZ

            self.axis['LX']  = msg.LX
            self.axis['LY']  = msg.LY
            self.axis['RX']  = msg.RX
            self.axis['RY']  = msg.RY
            self.axis['LSZ'] = msg.L2
            self.axis['RSZ'] = msg.R2

            self.read_timeout_count = 0
        except Exception as error:
            print(f"LCM Callback Error: {error}")

    def read_data(self):
        """
        Process LCM messages (non-blocking)
        """
        try:
            # self.lcm.handle_timeout(10)  # 10ms timeout
            # self.error_count = 0
            # return True
            val = self.lcm.handle_timeout(10)
            # print(val)
            if val == 0:
                self.error_count += 1
                return False
            elif val >= 1:
                self.error_count = 0
                return True
        except Exception as error:
            print(f"LCM Read Error: {error}")
            self.error_count += 1
            return False
    
    @staticmethod
    def print_message(message):
        """
        Print custom message
        """
        msg = '[THEMIS GAMEPAD | '

        # warning
        if message == 'connection_lost':
            msg += 'WARNING] :: Connection lost.'
            print(colored(msg, 'red'))
        elif message == 'connection_timeout':
            msg += 'WARNING] :: Connection timed out.'
            print(colored(msg, 'yellow'))
        elif message == 'send_timeout_count':
            msg += 'WARNING] :: Send timed out.'
            print(colored(msg, 'yellow'))
        elif message == 'read_timeout_count':
            msg += 'WARNING] :: Read timed out.'
            print(colored(msg, 'yellow'))
        elif message == 'bad_data':
            msg += 'WARNING] :: Data corrupted.'
            print(colored(msg, 'yellow'))

        # status
        elif message == 'connected':
            msg += 'STATUS]  :: Connected.'
            print(colored(msg, 'white'))
        elif message == 'disconnected':
            msg += 'STATUS]  :: Disconnected.'
            print(colored(msg, 'white'))
        elif message == 'connecting':
            msg += 'STATUS]  :: Connecting...'
            print(colored(msg, 'white'))
        elif message == 'reconnecting':
            msg += 'STATUS]  :: Reconnecting...'
            print(colored(msg, 'white'))