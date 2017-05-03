#!/usr/bin/python
import json
from i2ctest import move
import time

def annoy():
    move("right_eye", "lower_lid", 0.5)
    move("right_eye", "upper_lid", 0.5)
    move("right_eye", "pan", 0.5)
    move("right_eye", "tilt", 0.5)

    move("left_eye", "lower_lid", 0.5)
    move("left_eye", "upper_lid", 0.5)
    move("left_eye", "pan", 0.5)
    move("left_eye", "tilt", 0.5)

    time.sleep(0.5)
