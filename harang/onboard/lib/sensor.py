# 센서 통합 라이브러리

# BMP390(ALT), TEL0132(GPS), EBIMU-9DOFV5(IMU)


import time
import board
import adafruit_bmp3xx
import imu


i2c = board.I2C()
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)

PORT           = '/dev/ttyAMA0'
BAUD_HIGH      = 115200
BAUD_LOW       = 9600
SERIAL_TIMEOUT = 0.001

class Sensor:

    def __init__(self, port=PORT, baud=BAUD_LOW, timeout=SERIAL_TIMEOUT):
        self.sensor = None
        self._init_imu_low(port, baud, timeout)


    @property # TODO: 정의하기,,,,, reference/lib/sensor.py 참고
    def read(self):
        return BMP390.read(), TEL0132.read(), imu.read()



    def 



