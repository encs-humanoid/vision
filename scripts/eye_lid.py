import json
from i2cMessage import move
import time

def eye_lid(axis, which_eye):
    new_axis = (axis + 1) / 2

    if which_eye == "right_eye":
       move(which_eye, "lower_lid", new_axis)
       move(which_eye, "upper_lid", 1 - new_axis)
    elif which_eye == "left_eye":
       move(which_eye, "lower_lid", 1 - new_axis)
       move(which_eye, "upper_lid", new_axis)







