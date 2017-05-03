from ptp import ptpn1
from ptp import ptpn2
from ptp import defaultNeckPos
#from i2ctest import move
from i2cMessage import move
import time
import json



defaultNeckPos()
while 0 == 0:
    move("neck", "tilt", 0.8)
    ptpn1("pan", 0.5, 0.8, 0.2)
    move("neck", "tilt", 0.8)
    time.sleep(0.25)
    move("neck", "tilt", 0.8)
    ptpn1("pan", 0.8, 0.5, 0.2)
    move("neck", "tilt", 0.8)
    time.sleep(0.25)
    move("neck", "tilt", 0.8)
    ptpn1("pan", 0.5, 0.2, 0.2)    
    move("neck", "tilt", 0.8)
    time.sleep(0.25)
    move("neck", "tilt", 0.8)
    ptpn1("pan", 0.2, 0.5, 0.2)
    move("neck", "tilt", 0.8)
    time.sleep(0.25)
    ptpn1("roll", 0.5, 0.7, 0.2)
    time.sleep(0.25)
    ptpn1("roll", 0.7, 0.5, 0.2)
    time.sleep(0.25)
    ptpn1("roll", 0.5, 0.2, 0.2)
    time.sleep(0.25)
    ptpn1("roll", 0.2, 0.5, 0.2)
    time.sleep(0.25)
    
