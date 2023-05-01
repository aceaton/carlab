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

"""
THREAD ONE
drive the PWMs and scan for serial and hes inputs
"""



