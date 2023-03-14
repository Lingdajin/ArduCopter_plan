#!/usr/bin/envpython 
# -*- coding: utf-8-*- 
import RPi.GPIO as GPIO
import time

#设置TRIG ECHO口
TRIG = 23
ECHO = 24
#初始化
def distanceInit():
	print('Distance Measurement In Progress')
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(TRIG,GPIO.OUT)
	GPIO.setup(ECHO,GPIO.IN)

#测距主体
def distanceStart():
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

try:
        distanceInit()
	while True:
		distance = distanceStart()
		print("Distance:{}cm".format(distance))
		if distance < 100:
                    time.sleep(1)
except KeyboardInterrupt: #按下esc键退出
	GPIO.cleanup()


