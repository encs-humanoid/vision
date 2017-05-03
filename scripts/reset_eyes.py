#!/usr/bin/python
#from i2ctest import move
from i2cMessage import move
import time

def reseteye():
    move("right_eye", "lower_lid", 0.5)
    move("right_eye", "upper_lid", 0.5)
    move("right_eye", "pan", 0.5)
    move("right_eye", "tilt", 0.5)

    move("left_eye", "lower_lid", 0.5)
    move("left_eye", "upper_lid", 0.5)
    move("left_eye", "pan", 0.5)
    move("left_eye", "tilt", 0.5)


if __name__ == "__main__":
    reseteye()
    time.sleep(3)
