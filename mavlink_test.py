#!/usr/bin/envpython 
# -*- coding: utf-8-*- 
from __future__ import division, print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import threading

#dronekit-python初始化
connection_string ='192.168.238.237:14552' 
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
 
def send_local_ned_velocity(vx, vy, vz): #无人机为参考系控制速度
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,0,0,mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0,0,0,
        vx,vy,vz,
        0,0,0,
        0,0)
    
    for x in range(0,1):    
        vehicle.send_mavlink(msg)
        time.sleep(0.2)

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

arm_and_takeoff(2)
send_local_ned_velocity(1,0,0)
condition_yaw(1,30,1)