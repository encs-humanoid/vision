from i2ctest import move
from ptp import ptpn1
from ptp import ptpn2
from ptp import defaultNeckPos
import time
import json
import math

defaultNeckPos()
angle = 0

def findY(angle):
    y = math.cos(angle * (math.pi/180) )
    y = 0.5 * (y / 2) + 0.5
    return y

def findX(angle):
    x = math.sin(angle * (math.pi/180) )
    x = 0.5 * (x / 2) + 0.5
    return x

while 0 <= angle < 360:
    panValue = findX(angle)
    tiltValue = findY(angle)
    #move("neck", "pan", panValue)
    #move("neck", "tilt", tiltValue)
    print(panValue, tiltValue)
    time.sleep(0.05)
    angle += 15

    


