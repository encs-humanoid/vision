#!/usr/bin/python
import json
from i2cMessage import move
import time
from random import randint
import sys

def blink(num):
	for i in range(num):
    		move("right_eye", "lower_lid", 0)
    		move("right_eye", "upper_lid", 1)
    		move("left_eye", "lower_lid", 1)
    		move("left_eye", "upper_lid", 0)
    		time.sleep(0.3)

    		move("right_eye", "lower_lid", 0.5)
    		move("right_eye", "upper_lid", 0.5)
    		move("left_eye", "lower_lid", 0.5)
    		move("left_eye", "upper_lid", 0.5)
    		time.sleep(0.3)

if __name__ == "__main__":
	if len(sys.argv) > 1:
	    num = int(sys.argv[1])
	else:
	    num = 1
	blink(num)

