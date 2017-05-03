#!/usr/bin/python
import json
from i2cMessage import move
import math
import time
move("right_eye", "lower_lid", 0.1) 
move("right_eye", "upper_lid", 0.1)
move("left_eye", "lower_lid", 0.9) 
move("left_eye", "upper_lid", 0.9)
for i in range(2):
    for a in range(0, 360, 30):
	x = 0.3 * math.cos(a * math.pi/180.0) + 0.5
	y = 0.3 * math.sin(a * math.pi/180.0) + 0.5

	move("left_eye", "pan", x)
	move("left_eye", "tilt", y)
	move("right_eye", "pan", x)
	move("right_eye", "tilt", y)
	time.sleep(0.1)

move("left_eye", "pan", 0.5)
move("left_eye", "tilt", 0.5)
move("right_eye", "pan", 0.5)
move("right_eye", "tilt", 0.5)
