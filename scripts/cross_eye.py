#!/usr/bin/python
import json
from i2cMessage import move
import math
import time

move("left_eye", "pan", 1)
move("left_eye", "tilt", 0.2)
move("right_eye", "pan", 0)
move("right_eye", "tilt", 0.8)
time.sleep(1.0)

move("left_eye", "pan", 0.5)
move("left_eye", "tilt", 0.5)
move("right_eye", "pan", 0.5)
move("right_eye", "tilt", 0.5)
