import json
from i2cMessage import move
import time
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

def left_eye(x,y):
	p = (y * 0.5) + 0.5
	move("left_eye", "pan", 1 - p)
	t = (x * 0.5) + 0.5
	move("left_eye", "tilt", t)


def right_eye(x,y):
        p = (y * 0.5) + 0.5
	move("right_eye", "pan", 1 - p)
	t = (x * 0.5) + 0.5
	move("right_eye", "tilt", 1 - t)

