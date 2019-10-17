#!/usr/bin/python
import json
from i2cMessage import move
import math
import time

def roll_eyes_sync(n):
    for i in range(n):
	for a in range(0, 360, 30):
	    x = 0.5 * math.cos(a * math.pi/180.0) + 0.5
	    y = 0.5 * math.sin(a * math.pi/180.0) + 0.5

	    move("left_eye", "pan", x)
	    move("left_eye", "tilt", y)
	    move("right_eye", "pan", x)
	    move("right_eye", "tilt", y)
	    time.sleep(0.1)

    move("left_eye", "pan", 0.5)
    move("left_eye", "tilt", 0.5)
    move("right_eye", "pan", 0.5)
    move("right_eye", "tilt", 0.5)

if __name__ == "__main__":
    roll_eyes_sync(2)
