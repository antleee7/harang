# -*- coding: 949 -*-

import sys 
if hasattr(sys, "setdefaultencoding"): 
    sys.setdefaultencoding(sys.getfilesystemencoding()) 

from visual import *
import serial
import string
import math


grad2rad = 3.141592/180.0
rad2grad = 180.0/3.141592

comport_num = raw_input("COM Port : ")
comport_num = 'COM' + comport_num
comport_baudrate = input("Baudrate : ")
ser = serial.Serial(port=comport_num,baudrate=comport_baudrate)


# scene
scene=display(title="EBIMU-9DOF v1.0")
scene.range=(1.5,1.5,1.5)
scene.forward = (1,0,-0.16)
scene.up=(0,0,1)

scene.width = 800
scene.height = 800

# arrows
arrow(pos=(0.5,1,0),color=color.red,axis=(-0.5,0,0), shaftwidth=0.02, fixedwidth=1)
arrow(pos=(0.5,1,0),color=color.blue,axis=(0,-0.5,0), shaftwidth=0.02, fixedwidth=1)
arrow(pos=(0.5,1,0),color=color.green,axis=(0,0,0.5), shaftwidth=0.02, fixedwidth=1)


#Roll,Pitch,Yaw labels
label(pos=(0,1.0,-1.1),text="Roll",box=0,opacity=0)
label(pos=(0,0,-1.1),text="Pitch",box=0,opacity=0)
label(pos=(0,-1.0,-1.1),text="Yaw",box=0,opacity=0)
label(pos=(0,0,-0.1),text="Roll/Pith/Yaw",box=0,opacity=0)

#label(pos=(0,1.0,-1.1),text="Pitch",box=0,opacity=0)
#label(pos=(0,0,-1.1),text="Roll",box=0,opacity=0)
#label(pos=(0,-1.0,-1.1),text="Yaw",box=0,opacity=0)

L1 = label(pos=(0,1.0,-1.3),text="-",box=0,opacity=0 ,color=color.yellow)
L2 = label(pos=(0,0,-1.3),text="-",box=0,opacity=0,color=color.yellow)
L3 = label(pos=(0,-1.0,-1.3),text="-",box=0,opacity=0,color=color.yellow)

L_id = label(pos=(0,0,1.1),text="-",box=0,opacity=0,color=color.yellow)

# scene object
platform = box(pos=(0,0,0.5), length=0.6, height=0.1, width=1, material=materials.wood)
#platform=sphere(pos=(0,0,0.5), radius=0.5, material=materials.earth)

roll_stick = box(pos=(0,0.9,-0.7),length=0.3, height=0.5, width=0.05, color=color.red )
pitch_stick = box(pos=(0,0,-0.7),length=0.3, height=0.5, width=0.05, color=color.blue )
yaw_stick = box(pos=(0,-0.9,-0.7),length=0.3, height=0.5, width=0.05, color=color.green )


roll=0
pitch=0
yaw=0
old_roll=0
old_pitch=0
old_yaw=0

data_from = 1   # 1: sensor  2: rf_receiver
data_format = 1 # 1: euler   2: quaternion
data_index = 0



def quat_to_euler(x,y,z,w):
    euler = [0.0,0.0,0.0]
    
    sqx=x*x
    sqy=y*y
    sqz=z*z
    sqw=w*w
  
    euler[0] = asin(-2.0*(x*z-y*w)) 
    euler[1] = atan2(2.0*(x*y+z*w),(sqx-sqy-sqz+sqw))
    euler[2] = atan2(2.0*(y*z+x*w),(-sqx-sqy+sqz+sqw)) 

    return euler

while 1:
    line = ser.readline()
    words = string.split(line,",")    # Fields split

    if(-1 < words[0].find('*')) :
        data_from=1     # sensor data
        data_index=0
        L_id.text = "ID:"+'*'
        words[0]=words[0].replace('*','')
    else :
        if(-1 < words[0].find('-')) :
            data_from=2  # rf_receiver data
            data_index=1
            L_id.text = "ID:"+words[0]
        else :
            data_from=0  # unknown format


    if(data_from!=0):
        commoma = words[data_index].find('.') 
        if(len(words[data_index][commoma:-1])==4): # 소수부 4자리 판별
            data_format = 2  # quaternion
        else :
            data_format = 1 # euler


        if(data_format==1): #euler
            try:
                roll = float(words[data_index])*grad2rad
                pitch = float(words[data_index+1])*grad2rad
                yaw = float(words[data_index+2])*grad2rad
            except:
                print "."
        else: #(data_format==2)quaternion
            try:
                q0 = float(words[data_index])
                q1 = float(words[data_index+1])
                q2 = float(words[data_index+2])
                q3 = float(words[data_index+3])
                Euler = quat_to_euler(q0,q1,q2,q3)

                roll  = Euler[1]
                pitch = Euler[0]
                yaw   = Euler[2]
            except:
                print "."
        
            
        axis=(cos(pitch)*cos(yaw),-cos(pitch)*sin(yaw),sin(pitch)) 
        up=(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw),sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw),-cos(roll)*cos(pitch))
        platform.axis=axis
        platform.up=up
        platform.length=0.6
        platform.width=1

        pitch_stick.rotate( axis=(0,1,0), angle = old_pitch-pitch)
        old_pitch = pitch

        roll_stick.rotate( axis=(-1,0,0), angle = old_roll-roll )
        old_roll = roll

        yaw_stick.rotate( axis=(0,0,1), angle = old_yaw-yaw )
        old_yaw = yaw

        L1.text = "%.2f" %(roll*rad2grad)
        L2.text = "%.2f" %(pitch*rad2grad)
        L3.text = "%.2f" %(yaw*rad2grad)
ser.close

