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
# from lib.kalman import Kalman --> 칼만 라이브러리 만들어야 함.
from lib.comm import Comm
from lib.sensor import Sensor
from lib.gpio import Pin
from lib.quaternion import quaternion # 쿼터니언 관련 라이브러리



def rad2deg(rad):
    return rad * 57.2958

if __name__ == "__main__":


    launchready      = False # Launch Ready Mode에 진입하라는 명령을 받으면 True가 됩니다. Launch detection 이후에는 False가 됩니다.
    supervised       = False # 강제사출 명령을 받으면 True가 됩니다.
    
    launched         = False # Launch detection이후 True가 됩니다.
    
    maxDataRate      = False # maxDataRate으로 변경 이후 True가 됩니다.

    apogee           = False # Apogee 도달 이후 True가 됩니다.
    
    deploy           = False # PAYLOAD의 drogue deploy를 담당합니다.
    
    touchdown        = False # 로켓이 착륙했는지 판단합니다.
    
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
        radio.write(HOUSEKEEPING_DATA) SHIT # write그대로 쓰면 됨. 다만 어떤식으로 보낼지는 모르겠음.
        
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
            


        #######################################################################
        ## Do repeated actions (i.e. read from sensors) depending on latches
        #######################################################################
        
        if launchready: # Launch Ready mode가 시작됩니다.
            if maxDataRate is False:
                radio.maxDataRate # 한번만 실행되면 되는데, 굳이 while문 안에 넣을 필요...?
                maxDataRate = True
            telemetry_on = True # 지상국으로 Telemetry data를 보냅니다. (종료될때까지 유지되어야 하는데, 어떻게 구현하지? --> 반복적으로 실행되어야 하는 것들을 정리해놓고, 각각에 해당하는 상태들을 정의하면 될듯. 예를 들어 여기서 직접 telemetry data를 보내는 게 아니라 telemetry_write = True같이 해놓는거지. while문 안에 또 다른 if telemetry_write: 이런게 있는것이고. if 문을 병렬적으로 씁시다.)
            _logging_on = True # 마찬가지로 한번만 실행되면 됨... 'Telemetry Data Memory Writing Start'
            SUBSYSTEM_BAUD_RATE_INCREASE
            if sensor.imu_data["accel"] > 1.5:
                launched = True # Launch Recognition
                launchready = False # Launch Ready Mode를 종료합니다.
                
        if launched: # In-flight mode가 시작됩니다.
            ORIENTATION CHECK VIA IMU (KALMAN)
            if ALTITUDE < MAX_ALTITUDE - 1.0:
                apogee = True
            if ANGULAR_VELOCITY < 20DPS and (apogee or supervised) and FLIGHT_TIME > 20s:
                deploy = True # Drogue를 사출합니다.
                launched = False # In-flight mode를 종료합니다.
                
        if deploy: # Recovery mode가 시작됩니다.
            PAYLOAD DROGUE DEPLOY
            if PROXIMITY_SENSOR reads 'some threshold':
                if ORIENTATION_STABILIZED:
                    ROCKET DROGUE DEPLOY # TODO: 한 번만 실행되게끔.
            if GPS, INS READ TOUCHDOWN:
                touchdown = True
                deploy = False # Recovery mode가 종료됩니다.
                
        if touchdown:
            ALL SUBSYSTEMS DOWN EXCEPT COMM SYSTEM
            HOUSEKEEPING DATA DOWNLINK (MIN DATARATE)
                
                
###########DOWNLINK DATA를 병렬 if문으로 구현해야겠음.
        if telemetry_on: # 138번째 줄 telemetry if 문 구현 
            radio.write(TELEMETRY_DATA)
        
        # 각속도 감시
        if ANGULAR_VELOCITY > 20DPS:
            emergency = True # 전역변수 안써도 되나??
            
            
        if emergency: # 비상상황
            ALERT DATA DOWNLINK AND DEPLOYMENT COMMAND MONITOR
            APOGEE MONITOR VIA BAROMETER
            APOGEE MONITOR VIA GPS
            
            if or or :
                deploy = True
                emergency = False # 비상상황 종료
            
        
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
