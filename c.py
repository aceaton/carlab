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
# from angle_position import *
from tof_position import *
from position_utils import *

import pigpio

import serial
import time
import numpy as np

from digi.xbee.devices import XBeeDevice
from digi.xbee.packets.base import DictKeys
import sys
import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# from scipy.signal import find_peaks
from _peak_finding import find_peaks

# set up the GPIO in/out
import RPi.GPIO as GPIO
from time import sleep

pi = pigpio.pi()

# CONFIGURE GPIO
# motorpin = 12    
motorpin = 23            # PWM pin connected to LED
# GPIO.setwarnings(False)            #disable warnings
# GPIO.setmode(GPIO.BOARD)        #set pin numbering system
# GPIO.setup(motorpin,GPIO.OUT)
# pi_pwm = GPIO.PWM(motorpin,1000)        #create PWM instance with frequency
            #start PWM of required Duty Cycle 
pi.set_mode(motorpin, pigpio.OUTPUT)
pi.set_PWM_frequency(motorpin,100)
# gpioSetPullUpDown(18, PI_PUD_DOWN);
pi.set_pull_up_down(motorpin, pigpio.PUD_DOWN)

steerpin = 13
# GPIO.setup(steerpin,GPIO.OUT)
# steer_pwm = GPIO.PWM(steerpin,1000)
pi.set_mode(steerpin, pigpio.OUTPUT)
pi.set_PWM_frequency(steerpin,100)
# FIGURE THIS OUT
canpin = 22
pi.set_mode(canpin, pigpio.OUTPUT)
pi.set_PWM_frequency(canpin,100)


# GPIO.output(canpin, GPIO.LOW)
# can_pwm = GPIO.PWM(canpin,1000)
halleff_in = 17
# GPIO.setup(halleff_in, GPIO.IN)#, pull_up_down=GPIO.PUD_UP)

milli = 1e-3

device = XBeeDevice("/dev/ttyS0", 9600)



ser = serial.Serial(port="/dev/ttyACM0",baudrate=115200) # use dmesg to figure out USB

''' CONFIGURABLE PARAMS (don't change) ------------------------------------------------------'''
clp = .5
arr_size = 100 # must be divisible by 2
# PID coefficients (sc = speed control, st = steering)
sc_p = .3
sc_i = .000001
sc_d = 0 # half as much?
st_p = 0.008
st_i = 0
st_d = 0.008
# starting dcs
sc_starting_dc = 20
st_starting_dc = 15
can_dc = 20
# extreme dcs
sc_max_dc = 80
sc_min_dc = 3
st_max_dc = 80
st_min_dc = 0
# speed control
hes_per_foot = 7.5
target_speed = 1# in feet per second - sp=hes/second/7.5
# location stuff
loc_find_delay = 100
# location of transmitters (in feet units?)
t2_location = [14,8]
t3_location = [21,11]
t4_location = [25,6]

pwm_mult = 2.55
# pwm_mult = 10000

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
old_capture_val = 0
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
# can
can_counter = 0


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
                    pos_x = 3.28084*float(data[data.index("POS")+1])
                    pos_y = 3.28084*float(data[data.index("POS")+2])
                buffer_string = lines[-1]           

def halleff_callback(gpio,level,tick):
    # copy logic here

    # in the main func it's always doing the loc calc and pwm based on that at a regular interval
    # then it's changing target speed 
    # the duty cycle is pid controlled on target speed
    
    global rots
    global old_capture_val
    global sc_old_err
    global sc_int_err
    global arrived
        
    capture_val = time.time()
    rots +=1
    
    # capture
    diff = capture_val - old_capture_val
    old_capture_val = capture_val
    
    if (rots==1 or rots==2 or diff>2):
        diff = 1/target_speed/hes_per_foot # this is the target time between hes
        
    # error based on speed in f/s
    err = 1/(diff*hes_per_foot) - target_speed
    
    sc_int_err += err
    derr = err-sc_old_err
    sc_old_err = err
    print('speed: ' + str(1/(diff*hes_per_foot)) + ' err: ' + str(err) + ' interr: ' + str(sc_int_err) + ' derr: ' + str(derr))

    # PID
    calculated_dc = sc_starting_dc - (sc_p*err + sc_i*sc_int_err + sc_d*derr)
    if (calculated_dc > sc_max_dc):
         calculated_dc = sc_max_dc
    if (calculated_dc < sc_min_dc):
         calculated_dc = sc_min_dc
    print(int(calculated_dc*pwm_mult))
    if (not arrived):
        # print(calculated_dc)
        pi.set_PWM_dutycycle(motorpin, int(pwm_mult*calculated_dc))
        # pi_pwm.ChangeDutyCycle(int(1000000*calculated_dc))
    
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
    # print("GOT A PACKET")
    if i%1 == 0:
        packet_dict = packet.to_dict()
        api_data = packet_dict[DictKeys.FRAME_SPEC_DATA][DictKeys.API_DATA]
        rssi = api_data[DictKeys.RSSI]
        address16 = api_data[DictKeys.X16BIT_ADDR]
        end = time.time()
        # print("from: {}, RSSI: {}, Iteration: {}, Time: {}".format(address16, rssi, i, end-start))
        # if (idx2<100):
        #     # print(idx2)
        #     print("from: {}, RSSI: {}, Iteration: {}, Time: {}".format(address16, rssi, i, end-start))
            
        if (str(address16)=='00 02'):
            # print('2')/
            array2_lock.acquire()
            r2[idx2%arr_size] = rssi
            idx2+=1
            # print('2 ' + str(idx2))
            array2_lock.release()
        elif (str(address16)=='00 03'):
            array3_lock.acquire()
            r3[idx3%arr_size] = rssi
            # print('3')
            idx3+=1
            # print('3 ' + str(idx3))
            array3_lock.release()
        elif (str(address16)=='00 04'):
            array4_lock.acquire()
            r4[idx4%arr_size] = rssi
            # print('4')
            idx4+=1
            array4_lock.release()
            # print('4 ' + str(idx4))
        else:
            print(address16)

    if i == 100:
        i = 0
        #print(time.time() - start)

def get_location():
    if just_use_tag:
        return [pos_x,pos_y]
    else:
        return get_location_ang

def get_location_ang():
    array2_lock.acquire()
    array3_lock.acquire()
    array4_lock.acquire()
    idx = min([idx2,idx3,idx4])
    r2_c = np.copy(r2)
    r3_c = np.copy(r3)
    r4_c = np.copy(r4)
    array2_lock.release()
    array3_lock.release()
    array4_lock.release()
    r2_d = np.concatenate((r2_c[idx+1:],r2_c[:idx]))
    r3_d = np.concatenate((r3_c[idx+1:],r3_c[:idx]))
    r4_d = np.concatenate((r4_c[idx+1:],r4_c[:idx]))
    pos = get_pos_ang_3(r2_d,r3_d,r4_d,t2_location,t3_location,t4_location)
    return pos

# using two numpy arrays of same len
# gives t3 degrees clockwise from t2
def get_ang_2(r2,r3):

    # plt.close('all')
    a = np.clip(r2,np.amin(r2),(np.amax(r2)-np.amin(r2))*clp+np.amin(r2))
    b = np.clip(r3,np.amin(r3),(np.amax(r3)-np.amin(r3))*clp+np.amin(r3))
    b2 = np.flip(b[:int(arr_size/2)])
    r32 = np.flip(r3[:int(arr_size/2)])

    filt=np.convolve(a,b2,mode='valid')#full')[45:105]#[int(arr_size/2):int(arr_size)]  
    filt2=np.convolve(r2,r32,mode='valid')#[45:105]#[int(arr_size/2):int(arr_size)]  
    
    plt.figure(2)
    plt.plot(r2,label='r2')
    plt.plot(r3,label='r3')
    # plt.plot(a,label='r2')
    # plt.plot(b,label='r3')

    # # plt.plot(np.flip(b[:int(arr_size/2)]))
    plt.figure(0)
    plt.plot(filt)
    # plt.figure(1)
    # plt.plot(filt2)
    plt.show()
    # print("PLOTTED")

    ps1 = find_peaks(filt,distance=5,prominence=100)
    ps = ps1[0]
    print(len(ps))
    # ps = ps + len(b) - 1
    # print(ps)
    print(len)
    diff = np.mean(np.diff(ps))
    # ps[1]-ps[0]
    perc = 1-ps[0]/(diff)
    # print('t3 is ' + str(()*360) + ' degrees clockwise from t2')
    # print(ps1[1])
    print(perc)
    return perc*2*math.pi

def get_pos_ang_3(r2,r3,r4,p2,p3,p4):
    a1 = get_ang_2(r2,r3)
    a2 = get_ang_2(r3,r4)
    return calc_ang_pos(p2,p3,p4,a1,a2)

''' MAIN EXECUTION --------------------------------------------------------------------------'''
def s(): # main func start syst
    # pi_pwm.start(sc_starting_dc)
    # steer_pwm.start(st_starting_dc)
    pi.set_PWM_dutycycle(motorpin, int(pwm_mult*sc_starting_dc))
    pi.set_PWM_dutycycle(steerpin, int(pwm_mult*st_starting_dc))
    pi.set_PWM_dutycycle(canpin, int(pwm_mult*can_dc))
    # can_pwm.start(can_dc)

    # pi.set_PWM_enabled(motorpin, True)
    # pi.set_PWM_enabled(steerpin, True)
    # pi.set_PWM_enabled(canpin, True)
    
    # Create a new thread object
    xbee_thread = threading.Thread(target=threaded_xbee_interface)
    
    uwb_thread = threading.Thread(target=threaded_receive_position)
    uwb_thread.daemon = True
    xbee_thread.daemon = True
    uwb_thread.start()
    xbee_thread.start()

    pi.set_glitch_filter(halleff_in,100)
    pi.callback(halleff_in, pigpio.FALLING_EDGE, halleff_callback)

    # GPIO.add_event_detect(halleff_in, GPIO.FALLING, callback=halleff_callback, bouncetime=200)

def r(): # run
    while (1):
        # loop in here for pid for location
        # do pid
        global last_pos
        global st_old_err
        global st_int_err
        global arrived

        pos = get_location()

        # vec = [pos[0] - last_pos[0],pos[1]-last_pos[1]]
        # print(pos)
    
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
        
        if ( not arrived):
            pi.set_PWM_dutycycle(steerpin, int(pwm_mult*calculated_dc))

        # pi_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
        time.sleep(milli*loc_find_delay)
    
def cleanup():
    print("Exiting")
    ser.write("\r".encode())
    ser.close()
atexit.register(cleanup)