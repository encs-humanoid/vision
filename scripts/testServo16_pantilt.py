#!/usr/bin/env python
# This ROS Node converts Joystick inputs from the joy node
# into PiPan controls to manipulate a pan-tilt camera
# It publishes joint state messages.
# =======================================================
from __future__ import division
import atexit
from math import pi
import time
from Adafruit_PWM_Servo_Driver import PWM

i2c_control_mode = 0   # 0 - RPI GPI, 1 - Adafruit 16 channel I2C

##### pin assignments
pan_pin = 0
if (i2c_control_mode):
   tilt_pin = 1
else:
   tilt_pin =  1
pan_left_limit = 90
pan_right_limit = 210
tilt_down_limit = 125
tilt_up_limit = 185

pan_left_limit = 100
pan_right_limit = 220
tilt_down_limit = 190
tilt_up_limit = 130

try:
    if (i2c_control_mode):
       print "RPI i2c control mode"
       sb = open('/dev/servoblaster', 'w')
    else:
       print "servo16 control mode"
       servo16 = PWM(0x40)
       #servo16 = PWM(0x40, debug=True)

except (IOError):
   print "*** ERROR ***"
   print "Unable to communicate to the servo"
   exit()


# Pulse width is specified in 0.01 millisec values 
#   ie: a pulse value of 150 represents 150*.01 = 1.5 msec, which is center
def pwm( channel, pulse):
   if (i2c_control_mode):
      sb.write(str(channel) + '=' + str(int(pulse)) + '\n')
      sb.flush()

   else:
      print "pwm(chan=", channel, ", pulse=", pulse
      pulseLength = 1000000                   # 1,000,000 us per second
      pulseLength /= 60                       # 60 Hz
      pulseLength /= 4096                     # 12 bits of resolution
      pulse *= 10                             # convert value to usec
      pulse /= pulseLength                    # calculate channel pulse length
      print "servo16(chan=", channel, ", pulse=", pulse
      servo16.setPWM(channel, 0, int(pulse))

def go( pan_angle, tilt_angle):
	pwm(tilt_pin, map(tilt_angle, 0, 100, tilt_down_limit, tilt_up_limit))
	pwm(pan_pin, map(pan_angle, 0, 100, pan_left_limit, pan_right_limit))

def map(value, domainLow, domainHigh, rangeLow, rangeHigh):
    return ((value - domainLow) / (domainHigh - domainLow)) * (rangeHigh - rangeLow) + rangeLow

print "go center"
go(50, 30)
time.sleep(1)

print "go right"
go(0, 30)
time.sleep(1)

print "go left"
go(100, 30)
time.sleep(1)

print "go center"
go(50, 30)
time.sleep(1)

print "go up"
go(50, 100)
time.sleep(1)

print "go down"
go(50, 0)
time.sleep(1)

print "go center"
go(50, 30)

