#!/usr/bin/python

from __future__ import division
from Adafruit_PWM_Servo_Driver import PWM
import time

# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the PWM device using the default address
pwm = PWM(0x41)
# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)

servoMin = 150  # Min pulse length out of 4096
servoMax = 600  # Max pulse length out of 4096

def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  pulseLength /= 4096                     # 12 bits of resolution
  pulse *= 10				  # convert value to usec
  pulse /= pulseLength			  # calculate channel pulse length
  print "%d pulse" % int(pulse)
  pwm.setPWM(channel, 0, int(pulse))


if __name__ == '__main__':
    pwm.setPWMFreq(60)                        # Set frequency to 60 Hz

    for i in xrange(16):
       print i
       pwm.setPWM(i, 0, 0)
   
