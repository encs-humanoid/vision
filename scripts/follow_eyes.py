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
import i2cMessage
from i2cMessage import move
import vision.msg

from blink_both import blink
from roll_eye_sync import roll_eyes_sync
from annoyed import annoy
from glare import glaring
from blink_left import blinkLeft
from blink_right import blinkRight
from eyes_move import left_eye
from eyes_move import right_eye
from reset_eyes import reseteye
from joy_lid import lid
from random import randint
from threading import Timer

def callback(data):
    a = data.axes
    #x = map(a[0], -1, 1, loX, hiX)
    if a[1] <= 0.4 and a[1] >= -0.4:
    	move("right_eye", "tilt", 0.5)
	move("right_eye", "upper_lid", 0.1)
	move("right_eye", "lower_lid", 0.9)
	move("left_eye", "tilt", 0.5)
	move("left_eye", "upper_lid", 0.9)
	move("left_eye", "lower_lid", 0.1)
    #look down
    elif a[1] < -0.4:
	move("right_eye", "tilt", (-a[1] * 1.2) + 0.8)
	move("right_eye", "upper_lid", -a[1] * 0.8)
	move("right_eye", "lower_lid", -a[1] * 1.3)
	move("left_eye", "tilt", (a[1] * 1.2) - 0.8)
	move("left_eye", "upper_lid", a[1] * 1.8)
	move("left_eye", "lower_lid", a[1] * 0.3)

    #look up
    elif a[1] > 0.4:
        move("right_eye", "tilt", -a[1] * 0.2)
	move("right_eye", "upper_lid", -a[1] * 1.5)
	move("right_eye", "lower_lid", -a[1] + 1.8)
	move("left_eye", "tilt", a[1] * 1.2)
	move("left_eye", "upper_lid", a[1] * 1.5)
	move("left_eye", "lower_lid", a[1] - 1.4)

    move("right_eye", "pan", ((a[0] * 0.5) + 0.5))
    move("left_eye", "pan", ((a[0] * 0.5) + 0.5))
    
    if data.buttons[1] == 1:
    	blink(1)
	reseteye()
    if data.buttons[0] == 1:
	glaring()
	time.sleep(1)
	reseteye()
	roll_eyes_sync(1)
	glaring()
	time.sleep(3)
	blink(1)
    if data.buttons[6] == 1:
    	blinkLeft(1)
    if data.buttons[7] == 1:
        blinkRight(1)

class BlinkThread(threading.Thread):
    def __init__(self):
    	threading.Thread.__init__(self)
    def run(self):
        while(1):
	    r = randint(5,7)
	    time.sleep(r)
	    #blink(1)

class PanTiltConfig(object):
    def __init__(self):
	self.i2c_mode = 1
	self.pan_pin = 0
	self.tilt_pin = 1
	self.pan_left_limit = 90
	self.pan_right_limit = 210
	self.tilt_down_limit = 125
	self.tilt_up_limit = 185
	self.pan_center = 150
	self.tilt_center = 155

class PanTiltNode(object):
    def __init__(self):
	global publisher
	rospy.init_node('pan_tilt_node')

	config = PanTiltConfig()

	# I2C Mode: 0 - RPI GPI, 1 - Adafruit 16-channel I2C controller
	config.i2c_mode = int(self.get_param("i2c_mode", "1"))

	config.pan_pin = int(self.get_param("pan_pin", "0"))
	config.tilt_pin = int(self.get_param("tilt_pin", "1"))
	config.pan_left_limit = int(self.get_param("pan_left_limit", "90"))
	config.pan_right_limit = int(self.get_param("pan_right_limit", "210"))
	config.tilt_down_limit = int(self.get_param("tilt_down_limit", "125"))
	config.tilt_up_limit = int(self.get_param("tilt_up_limit", "185"))
	config.pan_center = int(self.get_param("pan_center", "150"))
	config.tilt_center = int(self.get_param("tilt_center", "155"))

	# publish joint states to sync with rviz virtual model
	# the topic to publish to is defined in the source_list parameter
	# as:  rosparam set source_list "['joints']"
	publisher = rospy.Publisher("/joints", JointState)
	i2cMessage.i2cthread = self

	# subscribed to joystick inputs on topic "joy"
	self.pub = rospy.Publisher("/i2cmove", vision.msg.I2CMove)
	rospy.Subscriber("/joy", Joy, callback)

	self.thread = BlinkThread()
	self.thread.setDaemon(True)
	self.thread.start()

    def get_param(self, param_name, param_default):
        value = rospy.get_param(param_name, param_default)
        rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param_name), value)
        return value


    def run(self):
	rospy.spin()

    def on_exit(self):
    	self.thread.join(1)
	rospy.loginfo("Exiting.")

    def publish(self, name, part, fraction):
        move = vision.msg.I2CMove()
	move.name = name
	move.part = part
	move.fraction = fraction
	self.pub.publish(move)



if __name__ == '__main__':
    try:
    	node = PanTiltNode()
	atexit.register(node.on_exit)
	node.run()
    except rospy.ROSInterruptException:
    	pass

