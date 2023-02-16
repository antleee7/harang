# HARANG Avionics - Telemetry

# 통신 관련 라이브러리입니다.

import serial
import time

PORT               = "/dev/ttyS0"
BAUD_HIGH          = 115200
BAUD_LOW           = 9600
SERIAL_TIMEOUT     = 0
TIME_BEFORE_RESEND = 0.5  # (s) how long to wait before panicking (send again)

class Comm:

    def __init__(self, port=PORT, baud=BAUD_LOW, timeout=SERIAL_TIMEOUT):
        self.radio = None
        self._init_comm_low(port, baud, timeout)

    def write(self, data):
        self.radio.write(data)

    def read(self, n_bytes=1):
        return self.radio.read(n_bytes)

    def bytesAvailable(self):
        return self.radio.inWaiting() # TODO: ?

    # Initialize low baudrate communication
    def _init_comm_low(self, port=PORT, baud=BAUD_LOW, timeout=SERIAL_TIMEOUT): 
        while True:
            try:
                self.radio = serial.Serial(port=port, baudrate=baud, timeout=timeout)
                print("Initialized low baudrate comm.")
                return True
            except serial.serialutil.SerialException:
                print("Radio not found, trying again. Did you run as `sudo`?")
                time.sleep(1)


    # Initialize high baudrate communication
    def _init_comm_high(self, port=PORT, baud=BAUD_HIGH, timeout=SERIAL_TIMEOUT): 
        while True:
            try:
                self.radio = serial.Serial(port=port, baudrate=baud, timeout=timeout)
                print("Initialized high baudrate comm.")
                return True
            except serial.serialutil.SerialException:
                print("Radio not found, trying again. Did you run as `sudo`?")
                time.sleep(1)
