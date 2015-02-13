#!/usr/bin/env python
# This ROS Node converts Joystick inputs from the joy node
# into PiPan controls to manipulate a pan-tilt camera
# It publishes joint state messages.
# =======================================================
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
import time
import threading
from math import pi

##### TODO Fix the pin assignments
left_tilt_pin = 0
right_tilt_pin = 1
pan_pin = 2

class PanTilt:
    def __init__(self):
        try:
            self.sb = open('/dev/servoblaster', 'w')
        except (IOError):
            print "*** ERROR ***"
            print "Unable to open the device, check that servod is running"
            print "To start servod, run: sudo /etc/init.d/servoblaster.sh start"
            exit()

    def pwm(self, pin, angle):
        self.sb.write(str(pin) + '=' + str(int(angle)) + '\n')
        self.sb.flush()

    def go(self, pan_angle, tilt_angle):
	self.pwm(left_tilt_pin, self.map_left_tilt_angle(tilt_angle))
	self.pwm(right_tilt_pin, self.map_right_tilt_angle(tilt_angle))
	self.pwm(pan_pin, pan_angle)

    def map_left_tilt_angle(angle):
	return map(angle, 0, 180, 0, 180)  #### TODO fix the angle mapping

    def map_right_tilt_angle(angle):
	return map(angle, 0, 180, 180, 0)  #### TODO fix the angle mapping

def map(value, domainLow, domainHigh, rangeLow, rangeHigh):
    return ((value - domainLow) / (domainHigh - domainLow)) * (rangeHigh - rangeLow) + rangeLow

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into pipan movement commands
# axis 1 aka left stick vertical controls the tilt servo
# axis 0 aka left stick horizonal controls the pan servo

# servo angle limits
loX = 0
hiX = 180
loY = 0
hiY = 180

speedFactor = 4
speedFactorX = -speedFactor
speedFactorY = speedFactor

# initial position and velocity
x0 = 90
y0 = 90
x = x0
y = y0
dx = 0
dy = 0
goCenter = False

def pan_tilt():
    global p, x, y, dx, dy, goCenter
    while True:
	if (goCenter):
	    x = x0
	    y = y0
	    p.go(x, y)
	    goCenter = False
	    publish_joint_state()
	elif (dx != 0 or dy != 0):
	    x += dx
	    y += dy
	    if (x < loX): x = loX
	    if (x > hiX): x = hiX
	    if (y < loY): y = loY
	    if (y > hiY): y = hiY
	    p.go(x, y)
	    publish_joint_state()
	# uncomment the else block to actively hold the position
	#else:
	#    p.go(x, y)
	time.sleep(0.05)

def publish_joint_state():
    pan_angle = (x0 - x) * pi / 180.0
    tilt_angle = 0.4618 * (y - y0) * pi / 180.0 # TODO correct this for the geometry of the robot

    print "x=" + str(x) + ", y=" + str(y) + ", pan=" + str(x-x0) + ", tilt=" + str(y-y0)
    #print "pan_angle=" + str(pan_angle) + ", tilt_angle=" + str(tilt_angle)

    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()

    joint_state.name.append('torso_neck_joint') # pan
    joint_state.position.append(pan_angle)
    joint_state.velocity.append(0)

    joint_state.name.append('upper_neck_head_joint') # tilt
    joint_state.position.append(tilt_angle)
    joint_state.velocity.append(0)

    publisher.publish(joint_state)

def callback(data):
    global dx, dy, speedFactorX, speedFactorY, goCenter
    a = data.axes
    #x = map(a[0], -1, 1, loX, hiX)
    #y = map(a[1], -1, 1, loY, hiY)
    dx = speedFactorX * a[0];
    dy = speedFactorY * a[1];
    if (data.buttons[8] == 1): # SELECT button pressed
	goCenter = True

# Intializes everything
def start():
    global p, publisher
    p = PanTilt()
    t = threading.Thread(target=pan_tilt)
    t.daemon = True
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # publish joint states to sync with rviz virtual model
    # the topic to publish to is defined in the source_list parameter
    # as:  rosparam set source_list "['joints']"
    publisher = rospy.Publisher("joints", JointState)
    # starts the node
    rospy.init_node('pipan_node')
    t.start()
    rospy.spin()

if __name__ == '__main__':
    start()
