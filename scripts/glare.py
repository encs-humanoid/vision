#!/usr/bin/python
import json
from i2cMessage import move
import time

def glaring():
    move("right_eye", "lower_lid", 0.9)
    move("right_eye", "upper_lid", 0.9)
    move("left_eye", "lower_lid", 0.1)
    move("left_eye", "upper_lid", 0.1)
    time.sleep(0.5)

if __name__ == "__main__":
    glaring()
    time.sleep(1)
