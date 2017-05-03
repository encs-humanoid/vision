#!/usr/bin/python
import json
from i2cMessage import move
import time

def blinkRight(n):
    for i in range(1):
	move("right_eye", "lower_lid", 0)
	move("right_eye", "upper_lid", 1)
	time.sleep(0.3)

	move("right_eye", "lower_lid", 1)
	move("right_eye", "upper_lid", 0)
	time.sleep(0.3)

