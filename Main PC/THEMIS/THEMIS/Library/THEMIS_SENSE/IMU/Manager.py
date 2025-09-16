#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 10, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Serial communication with IMU (https://www.microstrain.com/inertial-sensors/3dm-cv7-ahrs
                               https://s3.amazonaws.com/files.microstrain.com/CV7+Online/Home.htm
                               https://lord-microstrain.github.io/MSCL/Documentation/MSCL%20Documentation/index.html#
                               https://lord-microstrain.github.io/MSCL/Documentation/Getting%20Started/index.html?python#inertial
                               https://github.com/LORD-MicroStrain/MSCL)
'''

import time
import numpy as np
import Setting.robot_data as RDS
import Library.MATH_FUNCTION.math_function as MF
from termcolor import cprint
from Setting.Macros.constant_macros import *
from Library.THEMIS_SENSE.IMU.MSCL import mscl


class IMUSenseManager(object):
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, channel='AHRS_IMU'):
        # serial communication
        self.port = port
        self.baudrate = baudrate

        # imu states
        self.imu_data = {'scaledAccelX': 0.0, 'scaledAccelY': 0.0, 'scaledAccelZ': 0.0,
                         'scaledGyroX': 0.0, 'scaledGyroY': 0.0, 'scaledGyroZ': 0.0,
                         'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                         'estLinearAccelX': 0.0, 'estLinearAccelY': 0.0, 'estLinearAccelZ': 0.0,
                         'estCompensatedAccelX': 0.0, 'estCompensatedAccelY': 0.0, 'estCompensatedAccelZ': 0.0,
                         'estAngularRateX': 0.0, 'estAngularRateY': 0.0, 'estAngularRateZ': 0.0,
                         'estRoll': 0.0, 'estPitch': 0.0, 'estYaw': 0.0,
                         'estOrientationMatrix': np.eye(3)}
        self.imu_acceleration    = np.zeros(3)  # a_ii
        self.imu_angular_rate    = np.zeros(3)  # w_ii
        self.imu_euler_angles    = np.zeros(3)
        self.imu_rotation_matrix = np.eye(3)    # R_ni
        
        self.imu_yaw_offset = 0

        # base states
        self.base_acceleration    = np.zeros(3)  # a_bb
        self.base_angular_rate    = np.zeros(3)  # w_bb
        self.base_rotation_matrix = np.eye(3)    # R_wb

        # transformation ({w} -> {n} -> {i} -> {b})
        self.R_wn = MF.Rx(PI)
        self.R_ib = MF.Rx(PI)
        self.R_bi = self.R_ib.T

        # imu config
        self.is_streaming = False
        if channel in ['AHRS_IMU', 'ESTFILTER']:
            self.imu_channel = channel
        else:
            cprint("INVALID IMU CHANNEL!", 'red')
            exit()

        # initialize
        self.connect()
        self.configure()
        self.node.getDataPackets(100)  # clear buffer
        self.start_time = time.perf_counter()

        self.get_yaw_offset()
        self.node.getDataPackets(100)  # clear buffer

        self.idle()

    def connect(self):
        """
        Connect IMU
        """
        print("IMU connecting...")
        t0 = time.perf_counter()
        while True:
            # connection timeout
            if time.perf_counter() - t0 > 2:
                print("IMU offline.")
                raise RDS.IMU_OFFLINE

            try:
                self.connection = mscl.Connection.Serial(self.port, self.baudrate)  # create the connection object with port and baudrate
                self.node = mscl.InertialNode(self.connection)                      # create the InertialNode, passing in the connection
                if self.node.ping():
                    print("IMU connected.")
                    break
            except:
                pass

    def disconnect(self):
        """
        Disconnect IMU
        """
        self.idle()
        self.connection.disconnect()
        print("IMU disconnected.")

    def reconnect(self):
        """
        Reconnect IMU
        """
        self.disconnect()
        self.connect()

    def idle(self):
        """
        Set IMU to idle mode
        """
        self.node.setToIdle()
        self.is_streaming = False
        print("IMU in idle.")

    def enable_streaming(self):
        """
        Enable data streaming
        """
        if self.imu_channel == 'ESTFILTER':
            self.node.enableDataStream(mscl.MipTypes.CLASS_ESTFILTER, True)
            self.is_streaming = self.node.isDataStreamEnabled(mscl.MipTypes.CLASS_ESTFILTER)
        else:
            self.node.enableDataStream(mscl.MipTypes.CLASS_AHRS_IMU, True)
            self.is_streaming = self.node.isDataStreamEnabled(mscl.MipTypes.CLASS_AHRS_IMU)

        if self.is_streaming:
            print("IMU in streaming...")
        self.node.resume()

    def configure(self):
        """
        Configure IMU
        """
        if self.imu_channel == 'ESTFILTER':
            estFilterChs = mscl.MipChannels()
            # estFilterChs.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL, mscl.SampleRate.Hertz(1000)))  # excluding gravity
            estFilterChs.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_ESTFILTER_COMPENSATED_ACCEL,      mscl.SampleRate.Hertz(1000)))  # including gravity
            estFilterChs.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE, mscl.SampleRate.Hertz(1000)))
            estFilterChs.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER, mscl.SampleRate.Hertz(1000)))
            # estFilterChs.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_MATRIX, mscl.SampleRate.Hertz(1000)))
        else:
            ahrsImuChs = mscl.MipChannels()
            ahrsImuChs.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_SCALED_ACCEL_VEC, mscl.SampleRate.Hertz(1000)))
            ahrsImuChs.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_SCALED_GYRO_VEC,  mscl.SampleRate.Hertz(1000)))
            ahrsImuChs.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_EULER_ANGLES,     mscl.SampleRate.Hertz(1000)))
            
        try:
            # set the active channels
            if self.imu_channel == 'ESTFILTER':
                self.node.setActiveChannelFields(mscl.MipTypes.CLASS_ESTFILTER, estFilterChs)
            else:
                self.node.setActiveChannelFields(mscl.MipTypes.CLASS_AHRS_IMU,  ahrsImuChs)

            # disable absolute heading
            self.node.enableDisableAidingMeasurement(mscl.InertialTypes.MAGNETOMETER_AIDING, False)
            self.node.setInitialHeading(0)

            # x = mscl.GeometricVector(0, 0, 0)
            # self.node.setGyroBias(x)

            print("IMU configured.")
        except:
            pass

    def get_data(self):
        """
        Get IMU data
        """
        try:
            if not self.is_streaming:
                self.enable_streaming()

            packet = self.node.getDataPackets(0, 0)  # (timeout=0, max_packet_number=0, i.e., get all packets)
            if packet.size() > 0:
                points = packet[packet.size() - 1].data()
                if points.size() == 2:  # 1 data packet (of size 9) followed by 1 time packet (of size 2)!!!
                    if packet.size() > 1:
                        points = packet[packet.size() - 2].data()
                    else:
                        return False

                for point in points:
                    self.imu_data[point.channelName()] = point.as_float()

                if self.imu_channel == 'ESTFILTER':
                    # self.imu_acceleration[0] = self.imu_data['estLinearAccelX']
                    # self.imu_acceleration[1] = self.imu_data['estLinearAccelY']
                    # self.imu_acceleration[2] = self.imu_data['estLinearAccelZ']

                    self.imu_acceleration[0] = self.imu_data['estCompensatedAccelX']
                    self.imu_acceleration[1] = self.imu_data['estCompensatedAccelY']
                    self.imu_acceleration[2] = self.imu_data['estCompensatedAccelZ']

                    self.imu_angular_rate[0] = self.imu_data['estAngularRateX']
                    self.imu_angular_rate[1] = self.imu_data['estAngularRateY']
                    self.imu_angular_rate[2] = self.imu_data['estAngularRateZ']

                    self.imu_euler_angles[0] = self.imu_data['estRoll']
                    self.imu_euler_angles[1] = self.imu_data['estPitch']
                    self.imu_euler_angles[2] = self.imu_data['estYaw']

                    # omat = dataPoint[6].as_Matrix()
                    # self.imu_rotation_matrix = np.array([[omat.as_floatAt(0, 0), omat.as_floatAt(1, 0), omat.as_floatAt(2, 0)],
                    #                                      [omat.as_floatAt(0, 1), omat.as_floatAt(1, 1), omat.as_floatAt(2, 1)],
                    #                                      [omat.as_floatAt(0, 2), omat.as_floatAt(1, 2), omat.as_floatAt(2, 2)]])
                else:
                    self.imu_acceleration[0] = self.imu_data['scaledAccelX']
                    self.imu_acceleration[1] = self.imu_data['scaledAccelY']
                    self.imu_acceleration[2] = self.imu_data['scaledAccelZ']

                    self.imu_angular_rate[0] = self.imu_data['scaledGyroX']
                    self.imu_angular_rate[1] = self.imu_data['scaledGyroY']
                    self.imu_angular_rate[2] = self.imu_data['scaledGyroZ']

                    self.imu_euler_angles[0] = self.imu_data['roll']
                    self.imu_euler_angles[1] = self.imu_data['pitch']
                    self.imu_euler_angles[2] = self.imu_data['yaw']

                self.imu_rotation_matrix = MF.Rz(self.imu_euler_angles[2] - self.imu_yaw_offset) @ MF.Ry(self.imu_euler_angles[1]) @ MF.Rx(self.imu_euler_angles[0])
                self.imu2base()
                return True
            else:
                return False
        except:
            # print("IMU bad data!")
            return False

    def get_yaw_offset(self):
        """
        Get IMU yaw offset
        """
        print("Getting IMU yaw offset...")
        sample_number = 500
        count  = 0
        offset = 0
        self.imu_yaw_offset = 0
        while count < sample_number:
            if self.get_data():
                offset += self.imu_euler_angles[2] / sample_number
                count  += 1
        self.imu_yaw_offset = offset
        print("\033[1A", end="\x1b[2K")
        print("Getting IMU yaw offset... Done!")

    def imu2base(self):
        """
        Convert from IMU frame to base frame
        """
        # a_bb = R_bi @ a_ib = R_bi @ a_ii
        self.base_acceleration = self.R_bi @ self.imu_acceleration

        if self.imu_channel == 'AHRS_IMU':
            self.base_acceleration *= GRAVITY_ACCEL  # scaled accel unit is g

        # w_bb = R_bi @ w_ib = R_bi @ w_ii
        self.base_angular_rate = self.R_bi @ self.imu_angular_rate

        # R_wb = R_wn @ R_ni @ R_ib
        self.base_rotation_matrix = self.R_wn @ self.imu_rotation_matrix @ self.R_ib