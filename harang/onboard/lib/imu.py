import time
import serial
import RPi.GPIO as GPIO
from math import asin, atan2

PORT = '/dev/ttyAMA0'
BAUD_HIGH = 115200
BAUD_LOW = 9600
SERIAL_TIMEOUT = 0.001

grad2rad = 3.141592/180.0

class IMU:
    data_format = 1
    data_index = 0

    def __init__(self, port=PORT, baud=BAUD_LOW, timeout=SERIAL_TIMEOUT):
        self.imu = None
        self._init_imu_low(port, baud, timeout)

    def _init_imu_low(self, port=PORT, baud=BAUD_LOW, timeout=SERIAL_TIMEOUT):
        self.imu = serial.Serial(port=port, baud=baud, timeout=timeout)

    def _init_imu_high(self, port=PORT, baud=BAUD_HIGH, timeout=SERIAL_TIMEOUT):
        self.imu = serial.Serial(port=port, baud=baud, timeout=timeout)

    def quat_to_euler(self, x, y, z, w):
        euler = [0.0,0.0,0.0]
        sqx = x*x
        sqy = y*y
        sqz = z*z
        sqw = w*w
        euler[0] = asin(-2.0*(x*z-y*w))
        euler[1] = atan2(2.0*(x*y+z*w),(sqx-sqy-sqz+sqw))
        euler[2] = atan2(2.0*(y*z+x*w),(-sqx-sqy+sqz+sqw))
        return euler

    def main(self):
        line = self.imu.readline()
        words = line.split(",")

        commoma = words[IMU.data_index].find('.')
        if len(words[IMU.data_index][commoma:-1]) == 4:
            IMU.data_format = 2
        else:
            IMU.data_format = 1

        roll, pitch, yaw = 0, 0, 0

        if IMU.data_format == 1: # euler
            try:
                roll = float(words[IMU.data_index]) * grad2rad
                pitch = float(words[IMU.data_index+1]) * grad2rad
                yaw = float(words[IMU.data_index+2]) * grad2rad
            except ValueError:
                print

        return (roll, pitch, yaw)
