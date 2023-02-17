#  ___  ___  ________  ________  ________  ________   ________     
# |\  \|\  \|\   __  \|\   __  \|\   __  \|\   ___  \|\   ____\    
# \ \  \\\  \ \  \|\  \ \  \|\  \ \  \|\  \ \  \\ \  \ \  \___|    
#  \ \   __  \ \   __  \ \   _  _\ \   __  \ \  \\ \  \ \  \  ___  
#   \ \  \ \  \ \  \ \  \ \  \\  \\ \  \ \  \ \  \\ \  \ \  \|\  \ 
#    \ \__\ \__\ \__\ \__\ \__\\ _\\ \__\ \__\ \__\\ \__\ \_______\
#     \|__|\|__|\|__|\|__|\|__|\|__|\|__|\|__|\|__| \|__|\|_______|
#
# Onboard Flight Computer


# TODO: 센서 library 먼저 짜기 --> AIR.py에 data logging 코드 짜기 --> deploy mechanism --> 칼만 --> 기타 등등...
# @property --> 잘 쓰는 법...? library에 implement해야함.


import time
import serial
import os

import lib.logger as lgr
import numpy as np
from math import sqrt
from lib.stateuplink import State
# from lib.statedownlink import State --> TODO: 여기에 telemetry data패킷 형식도 포함해야 함
# from lib.kalman import Kalman --> TODO: 칼만 라이브러리 만들어야 함.
from lib.comm import Comm
from lib.sensor import BMP390 # TODO: 라이브러리 어떻게 할거임?
from lib.gpio import Pin
from lib.quaternion import quaternion # 쿼터니언 관련 라이브러리

import lib.imu
import lib.gps



def rad2deg(rad):
    return rad * 57.2958


max_alt = -inf # TODO 정의하기, 전역변수?? 코딩 잘 몰루..
t0 = time() # flight computer boot time
t1, t2 = 0, 0 # 시간 변수 초기화; t1: 발사 시각 t2: 경과 시각(while문에서 업데이트됨)

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
        if LOG_DEBUG: logger.write(text, lgr.DEBUG) # TODO: LOG_DEBUG --> 뭐하는 변수임?
            
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
    
    if radio._init_comm_high():
        debug("Radio max datarate nominal") # TODO: groundstation의 확인 필요?
        radio._init_comm_low() # 다시 low datarate로.
        
    # PDM and Memory Writing Check --> 'check'할 방법이 없으므로 패스. PDM은 사용하지 않기로 함

    debug("INITIALIZATION MODE FINISH")
    ###########################################################################
    ## INITIALIZATION MODE FINISH
    ###########################################################################
    
    ###########################################################################
    ## STANDBY MODE START
    ###########################################################################
    debug("STANDBY MODE START")
        
    sensor._init_low() # Subsystem baudrate decrease
    # TODO: subsystem baudrate decrease 굳이 필요함? sensor max baudrate check는 없는데?
    
    # Waiting for Mode Transition Command: 아래 while문에 구현했습니다.
    
    debug("ENTERING PROGRAM LOOP")
    while True:

        t2 = time() # t2 변수에 현재 시각 기록
        
        new_state = radio.read(); # 지상국의 명령을 받습니다.
        
        # Housekeeping data downlink (Low Data Rate)
        radio.write(sensor.read())
        
        # Battery Voltage and Temperature Check
        voltage = sensor.read() # TODO: sensor.py에서 read에서 thread형식 정하기.
        if voltage is good:
            debug('Battery Voltage Nominal: {} V'.format(voltage))
        else:
            debug('Battery Voltage has failed to reach expected values: {} V'.format(voltage))
        
        temp = sensor.read() # TODO
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
                radio._init_comm_high()
                maxDataRate = True
            telemetry_on = True
            _logging_on = True

            sensor._init_high() # subsystem baudrate increase

            if sensor.read()["accel"] > 1.5: # TODO: sensor read 정의하기!!!!
                launched = True # Launch Recognition
                t1 = time() # t1변수에 발사 시각을 기록합니다.
                launchready = False # Launch Ready Mode를 종료합니다.
                
        if launched: # In-flight mode가 시작됩니다.

            kalman = Kalman() # TODO: 라이브러리 만들기
            ORIENTATION CHECK VIA IMU (KALMAN) # TODO

            if sensor.read()["alt"] > max_alt:
                max_alt = sensor.read()["alt"]

            if sensor.read()["alt"] < MAX_ALTITUDE - 1.0:
                apogee = True

            # TODO: gyro data로 총 gps구하는 function
            if DPS < 20 and (apogee or supervised) and (t2 - t1) > 20.0:
                deploy = True # Drogue를 사출합니다.
                launched = False # In-flight mode를 종료합니다.
                
        if deploy: # Recovery mode가 시작됩니다.
            PAYLOAD DROGUE DEPLOY # TODO: GPIO pin 정의, high...?
            if PROXIMITY_SENSOR: # TODO: proxim sensor --> 1 또는 0밖에 출력값이 없음
                if ORIENTATION_STABILIZED: # TODO: 'stabilized' definition? any specific threshold?
                    ROCKET DROGUE DEPLOY # TODO: 사출 메커니즘
            if GPS, INS READ TOUCHDOWN: # TODO: touchdown 판별 메커니즘
                touchdown = True
                deploy = False # Recovery mode가 종료됩니다.
                
        if touchdown:
            ALL SUBSYSTEMS DOWN EXCEPT COMM SYSTEM # TODO: subsystem?
            HOUSEKEEPING DATA DOWNLINK (MIN DATARATE)
                
        
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

                # If local debugging is enabled, print to terminal directly. --> wtf?
                if LOCAL_DEBUG:
                    print ("R: %.2f  P: %.2f  Y: %.2f  "
                            "ACC_NORM: %.2f  ANGLE: %.2f  "
                            "FREEFALL: %s  APOGEE: %s" % (
                            rad2deg(data["fusionPose"][0]),
                            rad2deg(data["fusionPose"][1]), 
                            rad2deg(data["fusionPose"][2]),
                            accel_norm, theta, _freefall_detected, 
                            _apogee_detected))
