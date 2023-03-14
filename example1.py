#!/usr/bin/envpython 
# -*- coding: utf-8-*- 
from __future__ import print_function
import time 
from dronekit import connect, VehicleMode, LocationGlobalRelative 
 
 
# 当前连接的pixhawk飞控的端口 
connection_string ='/dev/ttyUSB0' #现在使用的是USB转ttl接口，连接pixhawk飞控
print('Connectingto vehicle on: %s' % connection_string) 

vehicle =connect(connection_string, wait_ready=True,baud=921600) 
 
# 定义arm_and_takeoff函数，使无人机解锁并起飞到目标高度 
# 参数aTargetAltitude即为目标高度，单位为米 
def arm_and_takeoff(aTargetAltitude): 
    # 起飞前检查 
    print("Basic pre-armchecks") 
    # vehicle.is_armable会检查飞控是否启动完成
    while not vehicle.is_armable: 
        print(" Waiting for vehicle toinitialise...") 
        time.sleep(1) 
 
    # 解锁无人机（电机将开始旋转） 
    print("Arming motors") 
    # 将无人机的飞行模式切换成"GUIDED"（一般建议在GUIDED模式下控制无人机） 
    vehicle.mode =VehicleMode("GUIDED") 
    # 通过设置vehicle.armed状态变量为True，解锁无人机 
    vehicle.armed = True 
 
    # 在无人机起飞之前，确认电机已经解锁 
    while not vehicle.armed:
        print(" Waiting forarming...") 
        time.sleep(1) 
 
    # 发送起飞指令 
    print("Taking off!") 
    # simple_takeoff将发送指令，使无人机起飞并上升到目标高度 
    vehicle.simple_takeoff(aTargetAltitude) 
 
    # 在无人机上升到目标高度之前，阻塞程序 
    while True: 
        print(" Altitude: ",vehicle.location.global_relative_frame.alt) 
        # 当高度上升到目标高度的0.95倍时，即认为达到了目标高度，退出循环 
        if vehicle.location.global_relative_frame.alt>= aTargetAltitude * 0.95: 
            print("Reached targetaltitude") 
            break 
        time.sleep(1) 
 
# 调用上面声明的arm_and_takeoff函数，目标高度10m 
arm_and_takeoff(10) 
 
 #停滞30秒
time.sleep(30) 
 
# 发送"返航"指令 
print("Returning to Launch") 
# 返航，只需将无人机的飞行模式切换成"RTL(Return to Launch)" 
# 无人机会自动返回home点的正上方，之后自动降落 
vehicle.mode =VehicleMode("RTL") 
 
# 退出之前，清除vehicle对象 
print("Closevehicle object") 
vehicle.close() 
