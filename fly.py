#!/usr/bin/envpython 
# -*- coding: utf-8-*- 
from __future__ import division, print_function
import time
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import RPi.GPIO as GPIO

center1=None
radius1=None
#摄像机初始化
lower_red=np.array([150,43,46])
upper_red=np.array([179,255,255])
cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)
#超声波初始化
TRIG = 23 #触发信号引脚
ECHO = 24 #回响信号引脚
print('Distance Measurement In Progress')
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

#dronekit-python初始化
connection_string ='192.168.177.237:14552' 
print('Connectingto vehicle on: %s' % connection_string) 
vehicle =connect(connection_string, wait_ready=True,baud=921600) 

#代码函数部分

def arm_and_takeoff(aTargetAltitude): #无人机起飞至指定高度
    # check pre-arm
    print("Basic pre-armchecks") 

    while not vehicle.is_armable: 
        print(" Waiting for vehicle toinitialise...") 
        time.sleep(1) 
 
    # arming
    print("Arming motors") 
    # change mode into GUIDED
    vehicle.mode =VehicleMode("GUIDED") 
    vehicle.armed = True # set arm=true
 
    # check arm-or-not again 
    while not vehicle.armed:
        print(" Waiting forarming...") 
        time.sleep(1) 
 
    # take off code 
    print("Taking off!") 
    vehicle.simple_takeoff(aTargetAltitude) 
 
    # checking the altitude
    while True: 
        print(" Altitude: ",vehicle.location.global_relative_frame.alt) 
        # when reach 0.95 of the altitude,stop the program
        #vehicle.location.global_relative_frame.alt     is the distance from home
        if vehicle.location.global_relative_frame.alt>= aTargetAltitude * 0.95: 
            print("Reached targetaltitude") 
            break 
        # wait for 1s 
        time.sleep(1) 
 
def distanceStart(): #超声波测距
	GPIO.output(TRIG,True)
	time.sleep(0.00001)
	GPIO.output(TRIG,False)

	while GPIO.input(ECHO) == 0:
		pass
	pulse_start = time.time()

	while GPIO.input(ECHO) == 1:
		pass
	pulse_end = time.time()

	pulse_duration = pulse_end - pulse_start
	distance = pulse_duration * 17150
	distance = round(distance,2)
	# print("Distance:{}cm".format(distance))
	return distance

def cam(): #摄像机识别
    ret,frame = cap.read()
    frame = cv2.GaussianBlur(frame,(5,5),0)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,lower_red,upper_red)

    mask = cv2.erode(mask,None,iterations=2)
    mask = cv2.GaussianBlur(mask,(3,3),0)
    res = cv2.bitwise_and(frame,frame,mask=mask)
    cnts = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) > 0:
        #找到面积最大的轮廓，并保存在c变量中。
        c=max(cnts,key=cv2.contourArea)
        #计算c的最小外接圆，并得到圆心和半径。
        ((x,y),radius)=cv2.minEnclosingCircle(c)
        #计算c的矩形边界框，并得到左上角坐标和宽高。
        rect=cv2.boundingRect(c)
        (x1,y1,w,h)=rect
        #计算c的几何中心，并得到坐标。
        M=cv2.moments(c)
        center=(int(M["m10"]/M["m00"]),int(M["m01"]/M["m00"]))
    
        #如果半径大于10，则执行以下代码：
        if radius > 10:
            #在frame上画出最小外接圆和矩形边界框，并用红色和绿色表示。
            cv2.circle(frame,(int(x),int(y)),int(radius),(0,0,255),2)
            cv2.rectangle(frame,(x1,y1),(x1+w,y1+h),(0,255,0),3)

            #返回圆心坐标和半径
            global center1
            center1 = center
            global radius1
            radius1 = radius

        else:
            #如果没有找到任何轮廓，则返回None,None
            return None,None
    else:
        pass
    cv2.imshow("frame",frame)

def send_global_ned_velocity(vx, vy, vz): #全局为参考系控制无人机坐标
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b110111000111,
        0,0,0,
        vx,vy,vz,
        0,0,0,
        0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
 
def send_local_ned_velocity(vx, vy, vz): #无人机为参考系控制坐标
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,0,0,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111000111,
        0,0,0,
        vx,vy,vz,
        0,0,0,
        0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def condition_yaw(direction,degrees, relative): #控制无人机航向
#direction needs to be 1 or -1,relative needs to be 1
    if relative:
        is_relative = 1
    else:
        is_relative = 0
    msg = vehicle.message_factory.command_long_encode(
        0,0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        degrees,
        0,
        direction,
        is_relative,
        0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

#代码运行部分

arm_and_takeoff(2) #Begin to take off to 2m

print('Open Camera')
while True: #Begin to open camera to analyse
    cam()
    distanceStart()
    if center1 is not None and radius1 is not None:
        x = center1[0]-320
        y = center1[1]-240
        if x < -10: #当红色中心位于视野中心左侧，无人机左偏航5度
            condition_yaw(-1,5,1)
        elif x > 10: #当红色中心位于视野中心右侧，无人机右偏航5度
            condition_yaw(1,5,1)
    if distance < 80:
        while distance >= 70:
            send_local_ned_velocity(1, 0, 0)
            if distance <= 50:
                break
        while distance <= 45:
            send_local_ned_velocity(-1, 0, 0)
            if distance >= 65:
                break
                
        
    if cv2.waitKey(5) & 0xFF == 27:
        break
print('Shut Down!')

print('Return Home Now')
vehicle.mode =VehicleMode("RTL") #return home

cap.release()
cv2.destroyAllWindows()
vehicle.close()
