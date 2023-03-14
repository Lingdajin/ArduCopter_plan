from __future__ import division, print_function
import time
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


lower_red=np.array([150,43,46])
upper_red=np.array([179,255,255])
cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

connection_string ='192.168.177.237:14552' 
print('Connectingto vehicle on: %s' % connection_string) 

vehicle =connect(connection_string, wait_ready=True,baud=921600) 

def arm_and_takeoff(aTargetAltitude): 
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
 

def cam(): #open camera
    ret,frame = cap.read()
    frame = cv2.GaussianBlur(frame,(5,5),0)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,lower_red,upper_red)

    mask = cv2.erode(mask,None,iterations=2)
    mask = cv2.GaussianBlur(mask,(3,3),0)
    res = cv2.bitwise_and(frame,frame,mask=mask)
    cnts = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) > 0:
        cnt = max(cnts,key=cv2.contourArea)
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        cv2.circle(frame,(int(x),int(y)),int(radius),(255,0,255),2)
        global num
        num = int(x)

    else:
        pass
    cv2.imshow("frame",frame)
    

#control the yaw
def condition_yaw(direction,degrees, relative): #direction needs to be 1 or -1,relative needs to be 1
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


arm_and_takeoff(2) #Begin to take off to 2m

print('Open Camera')
while True: #Begin to open camera to analyse
    num=600
    cam()
    if num >= 380: 
        condition_yaw(1,10,1)
        print(str(num)+'>380,turn right!')
    elif num <=160:
        condition_yaw(-1,10,1)
        print(str(num)+'<160,turn left!')
    if cv2.waitKey(5) & 0xFF == 27:
        break
print('Shut Down!')

print('Return Home Now')
vehicle.mode =VehicleMode("RTL") #return home

cap.release()
cv2.destroyAllWindows()
vehicle.close()
