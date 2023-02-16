# EBIMU-9DOFV2(IMU)의 센서값을 가져와서 읽을 수 있도록 변환해줍니다.

# IMU를 조건 없이 실행시키면, low baudrate에 따라 오일러각을 return합니다.

# imu high조건을 붙이고 실행하면 high baudrate에 따라 각을 return합니다.

import time
import serial
import RPi.GPIO as GPIO 


PORT           = '/dev/ttyAMA0'
BAUD_HIGH      = 115200
BAUD_LOW       = 9600
SERIAL_TIMEOUT = 0.001


class IMU:

    def __init__(self, port=PORT, baud=BAUD_LOW, timeout=SERIAL_TIMEOUT):
        self.imu = None
        self._init_imu_low(port, baud, timeout)


    def _init_imu_low(self, port=PORT, baud=BAUD_LOW, timeout=SERIAL_TIMEOUT):
        self.imu = serial.Serial(port=port, baud=baud, timeout=timeout)

    def _init_imu_high(self, port=PORT, baud=BAUD_HIGH, timeout=SERIAL_TIMEOUT):
        self.imu = serial.Serial(port=port, baud=baud, timeout=timeout)


    def main():
        transmitted_string = ""
        parsed_string  = ""
        i = 0
        while True:
            if imu.inWaiting():
                while imu.inWaiting():
                    transmitted_string += ser.read()
            sliced_string = transmitted_string[1:-2] # slice string
            splitted_string = sliced_string.split(",") # split with ','
        
            roll, pitch, yaw = float(splitted_string[0]), float(splitted_string[1]), float(splitted_string[2]) # 잘라놓은 데이터들을 실수로 파싱하여 변수에 저장

            for i in range(0, len(splitted_string)):
                parsed_string[i]= float(splitted_string[i])
                i+=1
            for x in parsed_string:
               print(x, end=' ')
            transmitted_string = ""
            parsed_string = ""
            i=0

            return roll, pitch, yaw

    if __name__ == "__main__":
        main()