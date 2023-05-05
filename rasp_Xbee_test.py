import numpy as np
from digi.xbee.devices import XBeeDevice
from digi.xbee.packets.base import DictKeys
import time 
import sys
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# from scipy.signal import find_peaks
from _peak_finding import find_peaks

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(23,GPIO.OUT)





milli = 1e-3

device = XBeeDevice("/dev/ttyS0", 9600)

#device.open()

#addr = device.get_16bit_addr()

#addr_int = int.from_bytes(addr, 'big')

#print(addr_int)
i = 0

arr_size = 200 # must be divisible by 2
r2 = np.zeros(arr_size)
r3 = np.zeros(arr_size)
idx2 = 0
idx3 = 0
clp = .5

try: 
    device.open()

    def packet_received_callback(packet):
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
            else:
                print(address16)

        if i == 100:
            i = 0
        #print(time.time() - start)

    device.add_packet_received_callback(packet_received_callback)

    while(1):
        time.sleep(milli)
        # if (idx2%array_size==0): 
        if ((idx3>=arr_size)and(idx2>=arr_size)):
            plt.close('all')
            # filt = np.convolve()
            # r2 = np.clip(r2,0,np.amax(r2))
            # r3 = np.clip(r3,0,np.amax(r3))
            a = np.clip(r2,np.amin(r2),(np.amax(r2)-np.amin(r2))*clp+np.amin(r2))
            b = np.clip(r3,np.amin(r3),(np.amax(r3)-np.amin(r3))*clp+np.amin(r3))
            b2 = np.flip(b[:int(arr_size/2)])
            r32 = np.flip(r3[:int(arr_size/2)])
            # b2 = b[:int(arr_size/2)]
            # r32 = r3[:int(arr_size/2)]
            filt=np.convolve(a,b2,mode='valid')#full')[45:105]#[int(arr_size/2):int(arr_size)]  
            filt2=np.convolve(r2,r32,mode='valid')#[45:105]#[int(arr_size/2):int(arr_size)]  
            plt.figure(2)
            plt.plot(r2,label='r2')
            plt.plot(r3,label='r3')
            plt.plot(a,label='r2')
            plt.plot(b,label='r3')
            # plt.plot(np.flip(b[:int(arr_size/2)]))
            plt.figure(0)
            plt.plot(filt)
            plt.figure(1)
            plt.plot(filt2)
            plt.show()
            print("PLOTTED")
            ps1 = find_peaks(filt,distance=10,prominence=600)
            ps = ps1[0]
            print(ps)
            # ps = ps + len(b) - 1
            # print(ps)
            diff = np.mean(np.diff(ps))
            # ps[1]-ps[0]
            print('t3 is ' + str((1-ps[0]/(diff))*360) + ' degrees clockwise from t2')
            print(ps1[1])
            
            

finally:
    if device is not None and device.is_open():
        device.close()
    