import json
from i2cMessage import move
import time
from blink_both import blink
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from Adafruit_PWM_Servo_Driver import PWM

def lid(a):	
	if a == 1:
		blink(1)

