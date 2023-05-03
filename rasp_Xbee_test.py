import numpy as np
from digi.xbee.devices import XBeeDevice
import time 
import sys

milli = 1e-3

device = XBeeDevice("/dev/ttyS0", 9600)

device.open()

addr = device.get_16bit_addr()

print(addr)

#while 1:
    #xbee_message = device.read_data()

    #remote = xbee_message.remote_device
    #remote.read_device_info()



    #data = xbee_message.data
    #is_broadcast = xbee_message.is_broadcast
    #timestamp = xbee_message.timestamp

    #time.sleep(milli)
