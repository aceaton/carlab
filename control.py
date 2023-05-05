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

from digi.xbee.devices import XBeeDevice
from digi.xbee.packets.base import DictKeys
import sys
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# from scipy.signal import find_peaks
from _peak_finding import find_peaks

# set up the GPIO in/out
import RPi.GPIO as GPIO
from time import sleep

# CONFIGURE GPIO
motorpin = 12				# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(motorpin,GPIO.OUT)
pi_pwm = GPIO.PWM(motorpin,1000)		#create PWM instance with frequency
			#start PWM of required Duty Cycle 
steerpin = 32
GPIO.setup(steerpin,GPIO.OUT)
steer_pwm = GPIO.PWM(steerpin,1000)
# FIGURE THIS OUT
# canpin = NUMBER
# GPIO.setup(canpin,GPIO.OUT)
# can_pwm = GPIO.PWM(canpin,1000)
halleff_in = 11
GPIO.setup(halleff_in, GPIO.IN)#, pull_up_down=GPIO.PUD_UP)

milli = 1e-3

device = XBeeDevice("/dev/ttyS0", 9600)

''' CONFIGURABLE PARAMS (don't change) '''
clp = .5
arr_size = 200 # must be divisible by 2

# PID coefficients (sc = speed control, st = steering)
sc_p = 0
sc_i = 0
sc_d = 0
st_p = 0
st_i = 0
st_d = 0

''' GLOBAL VARIABLES (change) '''
# 
i = 0
r2 = np.zeros(arr_size)
r3 = np.zeros(arr_size)
r4 = np.zeros(arr_size)
idx2 = 0
idx3 = 0
idx4 = 0

def start_system():
	pi_pwm.start(0)
	steer_pwm.start(0)
	# can_pwm.start(0)

	GPIO.add_event_detect(halleff_in, GPIO.FALLING, callback=halleff_callback, bouncetime=200)

	try: 
		device.open()
		device.add_packet_received_callback(packet_received_callback)

	while (1):
		# loop in here for pid for location
		# do pid

		pi_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100


def halleff_callback():
	# copy logic here

	# in the main func it's always doing the loc calc and pwm based on that at a regular interval
	# then it's changing target speed 
	# the duty cycle is pid controlled on target speed

def xbee_packet_received_callback(packet):

    global i 
    global r2
    global r3
    global idx2
    global idx3
    global arr_size
    i += 1
    start = time.time()
    if i%1 == 0:
        packet_dict = packet.to_dict()
        api_data = packet_dict[DictKeys.FRAME_SPEC_DATA][DictKeys.API_DATA]
        rssi = api_data[DictKeys.RSSI]
        address16 = api_data[DictKeys.X16BIT_ADDR]
        end = time.time()
        # print("from: {}, RSSI: {}, Iteration: {}, Time: {}".format(address16, rssi, i, end-start))
        if (idx2<100):
            print(idx2)
            print("from: {}, RSSI: {}, Iteration: {}, Time: {}".format(address16, rssi, i, end-start))
            
        if (str(address16)=='00 02'):
            # print('2')
            r2[idx2%arr_size] = rssi
            idx2+=1
        elif (str(address16)=='00 03'):
            r3[idx3%arr_size] = rssi
            # print('3')
            idx3+=1
		elif (str(address16)=='00 04'):
            r4[idx4%arr_size] = rssi
            # print('4')
            idx4+=1
        else:
            print(address16)

    if i == 100:
        i = 0
        #print(time.time() - start)

# if device is not None and device.is_open():
#         device.close()

pi_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100

