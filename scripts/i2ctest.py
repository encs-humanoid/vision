#!/usr/bin/python

from __future__ import division
from Adafruit_PWM_Servo_Driver import PWM
import json
import time

# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the PWM device using the default address
#pwm = PWM(0x40, debug=True)
#pwm.setPWMFreq(60)                        # Set frequency to 60 Hz
# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)

def newPWM(id):
    p = PWM(id, debug=True)
    p.setPWMFreq(60)
    return p

pwms = {
    0x40: newPWM(0x40),
    0x41: newPWM(0x41),
    0x42: newPWM(0x42)
}

def setServoPulse(channel, pulse):
    setServoPulsePWM(pwm, channel, pulse)

def setServoPulsePWM(p, channel, pulse):
    pulseLength = 1000000                   # 1,000,000 us per second
    pulseLength /= 60                       # 60 Hz
    pulseLength /= 4096                     # 12 bits of resolution
    pulse *= 10				  # convert value to usec
    pulse /= pulseLength			  # calculate channel pulse length
    print "%d pulse" % int(pulse)
    p.setPWM(channel, 0, int(pulse))

def go(board, channel, pulse):
    setServoPulsePWM(pwms[board], channel, pulse)

joint_data = None

def reload():
    global joint_data
    with open("joints.txt", "r") as f:
	joint_data = json.load(f)

def move(name, part, fraction):
    global joint_data
    if joint_data is None:
    	reload()
    if fraction > 1:
    	fraction = 1
    elif fraction < 0:
    	fraction = 0
    obj = joint_data[name][part]
    p = pwms[obj["board"]]
    channel = obj["channel"]
    if "cntr" in obj:
    	# calculate piecewise linear interpolation mapping 0.5 to the center
	if fraction < 0.5:
	    pulse = obj["llim"] + 2 * fraction * (obj["cntr"] - obj["llim"])
	elif fraction > 0.5:
	    pulse = obj["cntr"] + 2 * (fraction - 0.5) * (obj["ulim"] - obj["cntr"])
	else:
	    pulse = obj["cntr"]
    else:
	pulse = obj["llim"] + fraction * (obj["ulim"] - obj["llim"])
    print "Current Position: %f" % float(pulse)
    setServoPulsePWM(p, channel, pulse)

if __name__ == '__main__':
    while True:
	# read pulse
	try:
	    with open('pulse.txt') as f:
		pulses = f.readlines()

	    for channel in range(len(pulses)):
		pulse = eval(pulses[channel])
		setServoPulse(channel, pulse)
	except:
	    pass
