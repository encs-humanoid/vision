from i2cMessage import move
from ptp import ptpn2
import time
import json

move("neck", "roll", 0.5)
move("neck", "tilt", 0.5)
move("neck", "pan", 0.5)
ptpn2("tilt", 0.5, 0.2, "pan", 0.5, 0.2, 0.5)
ptpn2("tilt", 0.2, 0.8, "pan", 0.2, 0.8, 0.5)
ptpn2("tilt", 0.8, 0.2, "pan", 0.8, 0.2, 0.5)
ptpn2("tilt", 0.2, 0.5, "pan", 0.2, 0.5, 0.5)
