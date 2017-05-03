#!/usr/bin/env python
# This ROS Node converts Joystick inputs from the joy node
# into PiPan controls to manipulate a pan-tilt camera
# It publishes joint state messages.
# =======================================================
from __future__ import division
import atexit
from math import pi
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
import time
import threading
from Adafruit_PWM_Servo_Driver import PWM

class PanTiltConfig(object):
    def __init__(self):
	self.i2c_mode = 1
	self.pan_pin = 4
	self.tilt_pin = 5
	self.incline_pin = 6
	self.pan_left_limit = 90
	self.pan_right_limit = 210
	self.tilt_down_limit = 125
	self.tilt_up_limit = 185
	self.incline_left_limit = 140
	self.incline_right_limit = 170
	self.pan_center = 150
	self.tilt_center = 155
	self.incline_center = 155


class PanTilt(object):
    def __init__(self, config=PanTiltConfig()):
        try:
	    self.config = config
	    if (self.config.i2c_mode == 0):
	       self.sb = open('/dev/servoblaster', 'w')
	    else:
	       self.servo16 = PWM(0x40)

        except (IOError):
            print "*** ERROR ***"
            print "Unable to communicate to the servo"
            exit()


    # Pulse width is specified in 0.01 millisec values 
    #   ie: a pulse value of 150 represents 150*.01 = 1.5 msec, which is center
    def pwm(self, channel, pulse):
	 if (self.config.i2c_mode == 0):
	    self.sb.write(str(channel) + '=' + str(int(pulse)) + '\n')
	    self.sb.flush()
	 else:
	    print "pwm(chan=", channel, ", pulse=", pulse
	    pulseLength = 1000000                   # 1,000,000 us per second
	    pulseLength /= 60                       # 60 Hz
	    pulseLength /= 4096                     # 12 bits of resolution
	    pulse *= 10                             # convert value to usec
	    pulse /= pulseLength                    # calculate channel pulse length
	    print "servo16(chan=", channel, ", pulse=", pulse
	    self.servo16.setPWM(channel, 0, int(pulse))

    def go(self, pan_angle, tilt_angle, incline_angle):
	self.pwm(self.config.tilt_pin, map(tilt_angle, 0, 100, self.config.tilt_down_limit, self.config.tilt_up_limit))
	self.pwm(self.config.pan_pin, map(pan_angle, 0, 100, self.config.pan_left_limit, self.config.pan_right_limit))
	self.pwm(self.config.incline_pin, map(incline_angle, 0, 100, self.config.incline_left_limit, self.config.incline_right_limit))


def map(value, domainLow, domainHigh, rangeLow, rangeHigh):
    return ((value - domainLow) / (domainHigh - domainLow)) * (rangeHigh - rangeLow) + rangeLow


# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into pipan movement commands
# axis 1 aka left stick vertical controls the tilt servo
# axis 0 aka left stick horizonal controls the pan servo

# servo angle limits
loX = 0
hiX = 100
loY = 0
hiY = 100
loZ = 0
hiZ = 100

speedFactor = 4
speedFactorX = -speedFactor
speedFactorY = speedFactor
speedFactorZ = -speedFactor

# initial position and velocity
x0 = 50
y0 = 50
z0 = 50
x = x0
y = y0
z = z0
dx = 0
dy = 0
dz = 0

goCenter = False

def pan_tilt():
    global p, x0, y0, z0, x, y, z, dz, dx, dy, goCenter
    while True:
	if (goCenter):
	    x = x0
	    y = y0
	    z = z0
	    p.go(x, y, z)
	    goCenter = False
	    publish_joint_state()
	elif (dx != 0 or dy != 0 or dz != 0):
	    x += dx
	    y += dy
	    z += dz
	    if (x < loX): x = loX
	    if (x > hiX): x = hiX
	    if (y < loY): y = loY
	    if (y > hiY): y = hiY
            if (z < loZ): z = loZ
            if (z > hiZ): z = hiZ

	    p.go(x, y, z)
	    publish_joint_state()
	# uncomment the else block to actively hold the position
	else:
	    p.go(x, y, z)
	time.sleep(0.05)

def publish_joint_state():
    pan_angle = (x0 - x) * pi / 180.0
    tilt_angle = 0.4618 * (y - y0) * pi / 180.0 # TODO correct this for the geometry of the robot

    #print "x=" + str(x) + ", y=" + str(y) + ", pan=" + str(x-x0) + ", tilt=" + str(y-y0)
    #print "pan_angle=" + str(pan_angle) + ", tilt_angle=" + str(tilt_angle)

    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()

    joint_state.name.append('upper_neck_head_joint') # pan
    joint_state.position.append(pan_angle)
    joint_state.velocity.append(0)

    joint_state.name.append('torso_neck_joint') # tilt
    joint_state.position.append(tilt_angle)
    joint_state.velocity.append(0)

    publisher.publish(joint_state)

def callback(data):
    global dx, dy, dz, speedFactorX, speedFactorY, speedFactorZ, goCenter
    a = data.axes
    #x = map(a[0], -1, 1, loX, hiX)
    #y = map(a[1], -1, 1, loY, hiY)
    dx = speedFactorX * a[0];
    dy = speedFactorY * a[1];
    dz = speedFactorZ * a[3];
    if (data.buttons[8] == 1): # SELECT button pressed
	goCenter = True

class PanTiltNode(object):
    def __init__(self):
	global p, publisher
	rospy.init_node('pan_tilt_node')

	config = PanTiltConfig()

	# I2C Mode: 0 - RPI GPI, 1 - Adafruit 16-channel I2C controller
	config.i2c_mode = int(self.get_param("i2c_mode", "1"))

	config.pan_pin = int(self.get_param("pan_pin", "4"))
	config.tilt_pin = int(self.get_param("tilt_pin", "5"))
	config.incline_pin = int(self.get_param("incline_pin", "6"))
	config.pan_left_limit = int(self.get_param("pan_left_limit", "90"))
	config.pan_right_limit = int(self.get_param("pan_right_limit", "210"))
	config.tilt_down_limit = int(self.get_param("tilt_down_limit", "125"))
	config.tilt_up_limit = int(self.get_param("tilt_up_limit", "185"))
        config.incline_left_limit = int(self.get_param("incline_left_limit", "140"))
        config.incline_right_limit = int(self.get_param("incline_right_limit", "170"))
        config.incline_center = int(self.get_param("incline_center", "155"))
	config.pan_center = int(self.get_param("pan_center", "150"))
	config.tilt_center = int(self.get_param("tilt_center", "155"))
	p = PanTilt(config)

	# publish joint states to sync with rviz virtual model
	# the topic to publish to is defined in the source_list parameter
	# as:  rosparam set source_list "['joints']"
	publisher = rospy.Publisher("/joints", JointState)

	# subscribed to joystick inputs on topic "joy"
	rospy.Subscriber("/joy", Joy, callback)


    def get_param(self, param_name, param_default):
        value = rospy.get_param(param_name, param_default)
        rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param_name), value)
        return value


    def run(self):
	t = threading.Thread(target=pan_tilt)
	t.daemon = True
	t.start()
	rospy.spin()


    def on_exit(self):
    	rospy.loginfo("Exiting.")


if __name__ == '__main__':
    try:
    	node = PanTiltNode()
	atexit.register(node.on_exit)
	node.run()
    except rospy.ROSInterruptException:
    	pass

