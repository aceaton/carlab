import numpy as np
from digi.xbee.devices import XBeeDevice
from digi.xbee.packets.base import DictKeys
import time 
import sys

milli = 1e-3

device = XBeeDevice("/dev/ttyS0", 9600)

#device.open()

#addr = device.get_16bit_addr()

#addr_int = int.from_bytes(addr, 'big')

#print(addr_int)
i = 0

try: 
    device.open()

    def packet_received_callback(packet):
        global i 
        i += 1
        start = time.time()
        if i%5 == 0:
            packet_dict = packet.to_dict()
            api_data = packet_dict[DictKeys.FRAME_SPEC_DATA][DictKeys.API_DATA]
            rssi = api_data[DictKeys.RSSI]
            address16 = api_data[DictKeys.X16BIT_ADDR]
            end = time.time()
            print("from: {}, RSSI: {}, Iteration: {}, Time: {}".format(address16, rssi, i, end-start))
        if i == 100:
            i = 0
        #print(time.time() - start)

    device.add_packet_received_callback(packet_received_callback)

    while(1):
        time.sleep(milli)

finally:
    if device is not None and device.is_open():
        device.close()