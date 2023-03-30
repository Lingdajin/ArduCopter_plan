#!/usr/bin/envpython 
# -*- coding: utf-8-*- 
import cv2
import numpy as np
import time

#camera pre
lower_red = np.array([0, 100, 100])  # red
upper_red = np.array([10, 255, 255])
cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

def cam(): #摄像机识别
    ret,frame = cap.read()
    frame = cv2.GaussianBlur(frame,(5,5),0)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,lower_red,upper_red)

    mask = cv2.erode(mask,None,iterations=2)
    mask = cv2.GaussianBlur(mask,(3,3),0)
    res = cv2.bitwise_and(frame,frame,mask=mask)
    cnts = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    global center1
    global radius1
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
            center1 = center
            radius1 = radius

        else:
            #如果没有找到任何轮廓，则返回None,None
            center1 = [None,None]
            radius1 = None
    else:
        center1 = [None,None]
        radius1 = None
    cv2.imshow("frame",frame)
    time.sleep(0.1)

while True:
    cam()
    print ("x:",center1[0],"y:",center1[1])
    print ("半径：",radius1)
    if cv2.waitKey(1)&0xff==27:
        break

cap.release()
cv2.destroyAllWindows()