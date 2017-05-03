from ptp import ptpn1
from i2ctest import move
import time
import json

move("neck", "roll", 0.5)
move("neck", "tilt", 0.5)
move("neck", "pan", 0.5)
ptpn1("tilt", 0.5, 1, 1)
ptpn1("tilt", 1, 0, 1)
ptpn1("tilt", 0, 1, 1)
ptpn1("tilt", 1, 0.5, 1)
