# HARANG ONBOARD FLIGHT COMPUTER

# 하랑에 탑재될 컴퓨터의 코드입니다.

import time
import serial
import os

import lib.logger as lgr
import numpy as np
from math import sqrt
from lib.stateuplink import State
# from lib.statedownlink import State --> 여기에 telemetry data패킷 형식도 포함해야 함
from lib.comm import Comm
from lib.sensor import Sensor
from lib.gpio import Pin


def rad2deg(rad):
    return rad * 57.2958

if __name__ == "__main__":


    launchready      = False # Launch Ready Mode에 진입하라는 명령을 받으면 True가 됩니다.
    supervised       = False # 강제사출 명령을 받으면 True가 됩니다.
    
    launched         = False # Launch detection이후 True가 됩니다.
    
    emergency        = False # 각속도가 20dps가 넘는 비상상황시 True가 됩니다.


    # Set current state of air controller and declare the time we last sent a 
    # state update to the ground station
    state               = State()
    state_last_sent     = 0

    # Timing / counting variables
    launch_recognition          = 0
    apogee_counter              = 0
    flight_time                 = 0
    chute_ccd                   = 0
    touchdown                   = 0


    # Define logger but don't initialize a new log file here
    logger = lgr.Logger(init_log=False, init_camera=False, init_debug=True)

    # Define debug function
    def debug(text):
        if LOG_DEBUG: logger.write(text, lgr.DEBUG)
            
    ###########################################################################
    ## INITIALIZATION MODE START
    ###########################################################################
    debug("START: Initialization Mode")

    # INS/GPS Initialization
    sensor = Sensor()
    debug("Initialized INS/GPS.")
    sensor.start()
    
    # SDR Initialization and Communication Check
    radio = Comm()
    debug("Initialized Radio Communication.")
    
    if radio.maxDataRate():
        debug("Radio Max Data Rate Nominal.")
    else:
        debug("Radio has failed to reach maximum data rate.")
        
    # 여기서 다시 datarate를 low로 줄이는 부분 있어야 할듯?
        
    # PDM and Memory Writing Check --> 'check'할 방법이 없으므로 패스. PDM은 사용하지 않기로 함

    debug("INITIALIZATION MODE FINISH")
    ###########################################################################
    ## INITIALIZATION MODE FINISH
    ###########################################################################
    
    ###########################################################################
    ## STANDBY MODE START
    ###########################################################################
    debug("STANDBY MODE START")
        
    SUBSYSTEM BAUDRATE DECREASE # TODO: 정의하기
    
    # Waiting for Mode Transition Command: 아래 while문에 구현했습니다.
    
    debug("ENTERING PROGRAM LOOP")
    while True:
        
        new_state = radio.read(); # 지상국의 명령을 받습니다.
        
        # Housekeeping data downlink (Low Data Rate)
        radio.write(data) SHIT # write그대로 쓰면 됨. 다만 어떤식으로 보낼지는 모르겠음.
        
        # Battery Voltage and Temperature Check
        voltage = sensor.voltage()
        if voltage is good:
            debug('Battery Voltage Nominal: {} V'.format(voltage))
        else:
            debug('Battery Voltage has failed to reach expected values: {} V'.format(voltage))
        
        temp = sensor.temperature
        if temp is good:
            debug('Temperature Nominal: {} degrees C'.format(temp))
        else:
            debug('Temperature has failed to reach expected values: {} degrees C'.format(temp))
            
    ###########################################################################
    ## STANDBY MODE FINISH
    ###########################################################################
        # 지상국의 명령을 받습니다.
        if new_state != "":
            new_state = ord(new_state)  # convert from char to int
            debug("New state read (%d)." % new_state)

            # Get bit flags from new state
            launchready  = state.get_bit(state.LAUNCH_BIT, byte=new_state)
            supervised   = state.get_bit(state.DEPLOY_BIT, byte=new_state)

            if launchready:
                PROCEED TO LAUNCH READY MODE
            if supervised:
                DEPLOY DROGUE CHUTE

        #######################################################################
        ## Do repeated actions (i.e. read from sensors) depending on latches
        #######################################################################
        
        # Emergency Recovery Mode
        if ANGULAR VELOCITY > 20DPS:
            emergency = True # 전역변수 안써도 되나??
        
        # If logging is on, write IMU data to logfile! TODO: We have yet to 
        # implement sensor logging from the BMP280 because its read speed is 
        # slower than from the IMU and requires dedicated logic.
        if _logging_on:
            # Get most recently read data from the IMU
            data = sensor.imu_data
            if data is not None:
                t = time.time() - t0
                debug("[%s] Data read" % t)
                data_vector = [t, state.state,
                    data["fusionPose"][0], data["fusionPose"][1], data["fusionPose"][2], # fusionPose=각도인듯
                    data["compass"][0],    data["compass"][1],    data["compass"][2],
                    data["accel"][0],      data["accel"][1],      data["accel"][2],
                    data["gyro"][0],       data["gyro"][1],       data["gyro"][2]]
                logger.write(data_vector)

                # If local debugging is enabled, print to terminal directly.
                if LOCAL_DEBUG:
                    print ("R: %.2f  P: %.2f  Y: %.2f  "
                            "ACC_NORM: %.2f  ANGLE: %.2f  "
                            "FREEFALL: %s  APOGEE: %s" % (
                            rad2deg(data["fusionPose"][0]),
                            rad2deg(data["fusionPose"][1]), 
                            rad2deg(data["fusionPose"][2]),
                            accel_norm, theta, _freefall_detected, 
                            _apogee_detected))
