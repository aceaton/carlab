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

''' IMPORTS ---------------------------------------------------------------------------------'''
# generic stuff
import logging, threading, time, atexit
# custom stuff
from angle_position import *
from tof_position import *
from position_utils import *

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
motorpin = 12                # PWM pin connected to LED
GPIO.setwarnings(False)            #disable warnings
GPIO.setmode(GPIO.BOARD)        #set pin numbering system
GPIO.setup(motorpin,GPIO.OUT)
pi_pwm = GPIO.PWM(motorpin,1000)        #create PWM instance with frequency
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

ser = serial.Serial(port="/dev/ttyACM0",baudrate=115200) # use dmesg to figure out USB

''' CONFIGURABLE PARAMS (don't change) ------------------------------------------------------'''
clp = .5
arr_size = 200 # must be divisible by 2
# PID coefficients (sc = speed control, st = steering)
sc_p = 60
sc_i = 10
sc_d = 5 # half as much?
st_p = 0.008
st_i = 0
st_d = 0.008
# starting dcs
sc_starting_dc = 8
st_starting_dc = 6
can_dc = 45
# extreme dcs
sc_max_dc = 65
sc_min_dc = 0
st_max_dc = 80
st_min_dc = 0
# speed control
hes_per_foot = 7.5
target_speed = 1 # in feet per second - sp=hes/second/7.5
# location stuff
loc_find_delay = 100
# location of transmitters (in feet units?)
t2_location = [0,100]
t3_location = [100,100]
t4_location = [50,0]

''' GLOBAL VARIABLES (do change) ------------------------------------------------------------'''
# stuff for array capturing
i = 0
r2 = np.zeros(arr_size)
r3 = np.zeros(arr_size)
r4 = np.zeros(arr_size)
idx2 = 0
idx3 = 0
idx4 = 0
array2_lock = threading.Lock()
array3_lock = threading.Lock()
array4_lock = threading.Lock()
# stuff for speed control
rots = 0
sc_old_capture_val = 0
sc_old_err = 0
sc_int_err = 0
# stuff for position control
last_pos = [0,0]
targ_pos = [50,50]
mag_multiplier = .5 # half of the correction is influenced by distnace error (at a base distance of 1 ft)
st_old_err = 0
st_int_err = 0
arrived=False
# ttof calc pos
just_use_tag = True
pos_x=0
pos_y=0

''' HELPERS AND CALLBACK FUNCTIONS ----------------------------------------------------------'''                
def threaded_xbee_interface():
    try: 
        device.open()
        device.add_packet_received_callback(xbee_packet_received_callback)
        while (1):
            time.sleep(milli)
    finally:
        if device is not None and device.is_open():
            device.close()

def threaded_receive_position():
    global pos_x
    global pos_y
    # setup board deacawave
    ser.write("\r\r".encode()) # enter UART shell mode
    time.sleep(1)
    ser.write("lep\r".encode()) # start recieveing 
    time.sleep(1)
    
    buffer_string = ''
    while True:
        buffer_string = buffer_string + ser.read(ser.inWaiting()).decode()
        if '\n' in buffer_string:
            lines = buffer_string.split('\n') # guaranteed at least 2
            if lines[-2]:
                data = lines[-2]
                data = data.replace("\r\n","")
                data = data.split(",")
                if ("POS" in data):
                    pos_x = data[data.index("POS")+1]
                    pos_y = data[data.index("POS")+2]
                buffer_string = lines[-1]           

def halleff_callback():
    # copy logic here

    # in the main func it's always doing the loc calc and pwm based on that at a regular interval
    # then it's changing target speed 
    # the duty cycle is pid controlled on target speed
    
    global rots
    global old_capture_val
    global sc_old_err
    global sc_int_err
        
    capture_val = time.time()
    rots +=1
    
    # capture
    diff = capture_val - old_capture_val
    old_capture_val = capture_val
    
    if (rots==1 or rots==2):
        diff = 1/target_speed/hes_per_foot # this is the target time between hes
        
    # error based on speed in f/s
    err = 1/(diff*hes_per_foot) - target_speed
    sc_int_err += err
    derr = err-sc_old_err
    sc_old_err = err

    # PID
    calculated_dc = sc_starting_dc - (sc_p*err + sc_i*sc_int_err + sc_d*derr)
    if (calculated_dc > sc_max_dc):
         calculated_dc = sc_max_dc
    if (calculated_dc < sc_min_dc):
         calculated_dc = sc_min_dc
    if (not arrived)
        pi_pwm.ChangeDutyCycle(calculated_dc)
    
def xbee_packet_received_callback(packet):
    global i 
    global r2
    global r3
    global r4
    global idx2
    global idx3
    global idx4
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
            array2_lock.acquire()
            r2[idx2%arr_size] = rssi
            idx2+=1
            array2_lock.release()
        elif (str(address16)=='00 03'):
            array3_lock.acquire()
            r3[idx3%arr_size] = rssi
            # print('3')
            idx3+=1
            array3_lock.release()
        elif (str(address16)=='00 04'):
            array4_lock.acquire()
            r4[idx4%arr_size] = rssi
            # print('4')
            idx4+=1
            array4_lock.release()
        else:
            print(address16)

    if i == 100:
        i = 0
        #print(time.time() - start)

def get_location():
    if just_use_tag:
        return [pos_x,pos_y]
    array2_lock.acquire()
    array3_lock.acquire()
    array4_lock.acquire()
    idx = min([idx2,idx3,idx4])
    r2_c = r2.copy(r2)
    r3_c = r3.copy(r3)
    r4_c = r4.copy(r4)
    array2_lock.release()
    array3_lock.release()
    array4_lock.release()
    r2_d = r2_c[idx+1:]+r2_c[:idx]
    r3_d = r3_c[idx+1:]+r3_c[:idx]
    r4_d = r4_c[idx+1:]+r4_c[:idx]
    pos = get_pos_ang_3(r2_d,r3_d,r4_d,t2_location,t3_location,t4_location)
    return pos

''' MAIN EXECUTION --------------------------------------------------------------------------'''
def start_system(): # main func
    pi_pwm.start(sc_starting_dc)
    steer_pwm.start(st_starting_dc)
    # can_pwm.start(can_dc)
    
    # Create a new thread object
    xbee_thread = threading.Thread(target=threaded_xbee_interface)
    
    uwb_thread = threading.Thread(target=threaded_receive_position)
    uwb_thread.daemon = True
    xbee_thread.daemon = True
    uwb_thread.start()
    xbee_thread.start()
    
    GPIO.add_event_detect(halleff_in, GPIO.FALLING, callback=halleff_callback, bouncetime=200)

    while (1):
        # loop in here for pid for location
        # do pid
        global last_pos
        global st_old_err
        global st_int_err

        pos = get_location()
        # vec = [pos[0] - last_pos[0],pos[1]-last_pos[1]]
        
    
        # tvec = [targ_pos[0] - pos[0],targ_pos[1]-pos[1]]
        mag = d(pos,last_pos)
    
        mag_to_go = d(targ_pos,pos)
        if (mag_to_go < 1):
            arrived=True
    
        ang_err = ang(last_pos,pos) - ang(pos,targ_pos)
                
        last_pos = pos
                
        mult = 1 - mag_multiplier + mag*mag_multiplier # multiplier to take into account speed ish
        # could also just use speed directly
        
        # PID
        st_int_err += ang_err
        derr = ang_err-st_old_err
        st_old_err = ang_err

        # PID
        calculated_dc = st_starting_dc - (st_p*ang_err + st_i*st_int_err + st_d*derr)
        if (calculated_dc > st_max_dc):
             calculated_dc = st_max_dc
        if (calculated_dc < st_min_dc):
             calculated_dc = st_min_dc
        
        if ( not arrived)
            steer_pwm.ChangeDutyCycle(calculated_dc)

        # pi_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
        time.sleep(milli*loc_find_delay)
    
def cleanup():
    print("Exiting")
    ser.write("\r".encode())
    ser.close()
atexit.register(cleanup)