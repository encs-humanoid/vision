#!/usr/bin/python
import json
from i2cMessage import move
import time

def blinkLeft(n):
    for i in range(n):
	move("left_eye", "lower_lid", 1)
	move("left_eye", "upper_lid", 0)
	time.sleep(0.3)

	move("left_eye", "lower_lid", 0)
	move("left_eye", "upper_lid", 1)
	time.sleep(0.3)

