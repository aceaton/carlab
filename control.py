"""
CAR CONTROLLER, Anna Eaton and Christian Skinker, carlab spring 2023
runs on a raspberry pi

I/O specs:
Inputs: 
- Angle receiver via xbee - through gpio pins
- tof (time of flight) receiver - through USB
- Hall effect sensor - through gpio

Outputs (all digital through GPIO):
- PWM signal for speed control
- PWM signal for turn control
- digital signal for h-bridge - forwards and backwards

Imported custom modules:
- angle_position - uses the xbee network to calculate position
    - get_pos
- tof_position - uses the 
    - get_pos

- there will be a main func which is constantly interfacing w angle in, tof in, and gpio for hes, and updating those values
it will also interface outwards by sending pwm signals to the motor and servo
- there will be another control thread that will use the values of these inputs to do computation and then set the values of the PWM duty cycle, etc
it will require interrupts from the constant thread
"""

# IMPORTS
# generic stuff
import logging, threading, time
# custom stuff
import angle_position, tof_position

import serial
import time
import numpy as np

import serial
import RPi.GPIO as GPIO

"""
THREAD ONE
drive the PWMs and scan for serial and hes inputs
"""



# angle loc info (a_ prefix)
a_port = 0
a_ser = serial.Serial('/dev/ttyACM'+str(port),9600,timeout=5)#8,'N',1,timeout=1)

while True:
	line = ser.readline()
	if len(line) == 0:
		print("Timeout")
		sys.exit()
	print(line)

# read from io and deplex and save to arrays
# 3 transmitter case
def get_angle_arrs_3(window=100):
	r0 = np.empty(window,dtype=int)
	r1 = np.empty(window,dtype=int)
	r2 = np.empty(window,dtype=int)
	for (i=0;i<window;i+=1):
		line = ser.readline()
		if len(line) ==0:
			print("angle receiver timeout")
			sys.exit()
		r0[i] = line
		line = ser.readline()
		
