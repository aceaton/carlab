import RPi.GPIO as GPIO
from time import sleep

motorpin = 12				# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(motorpin,GPIO.OUT)
pi_pwm = GPIO.PWM(motorpin,1000)		#create PWM instance with frequency
pi_pwm.start(0)				#start PWM of required Duty Cycle 

steerpin = 32
GPIO.setup(steerpin,GPIO.OUT)
steer_pwm = GPIO.PWM(steerpin,1000)
steer_pwm.start(0)

def gpio_setup():
    import RPi.GPIO as GPIO
    from time import sleep
    motorpin = 12				# PWM pin connected to LED
    GPIO.setwarnings(False)			#disable warnings
    GPIO.setmode(GPIO.BOARD)		#set pin numbering system
    GPIO.setup(motorpin,GPIO.OUT)
    pi_pwm = GPIO.PWM(motorpin,1000)		#create PWM instance with frequency
    pi_pwm.start(0)				#start PWM of required Duty Cycle 

    steerpin = 32
    GPIO.setup(steerpin,GPIO.OUT)
    steer_pwm = GPIO.PWM(steerpin,1000)
    steer_pwm.start(0)


while True:
    # for duty in range(0,101,1):
    for duty in range(0,50,1):
        pi_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
        sleep(0.01)
        print(duty)
    sleep(0.5)
    for duty in range(50,-1,-1):
        pi_pwm.ChangeDutyCycle(duty)
        sleep(0.01)
        print(duty)
    sleep(0.5)
    