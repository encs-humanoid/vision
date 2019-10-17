#!/usr/bin/env python
# This ROS Node listens for joystick messages and sends commands to move joints
# to the /i2cmove topic.
#==================================================================================
from __future__ import division
import atexit
import i2cMessage
from i2cMessage import move
import json
import os.path
from math import pi
import rospy
from sensor_msgs.msg  import Joy
from sensor_msgs.msg import JointState
import std_msgs.msg
import vision.msg
import threading
import time
from blink_both import blink
from roll_eye_sync import roll_eyes_sync
from glare import glaring
from blink_left import blinkLeft
from blink_right import blinkRight
from reset_eyes import reseteye
from random import randint
from eye_lid import eye_lid

class JoyControl(object):
    def __init__(self):
	rospy.init_node('joy_control')

	self.controls = None
	
	self.init_controls()

	rospy.Subscriber("/joy", Joy, self.on_joy)
	rospy.Subscriber("/control", std_msgs.msg.String, self.on_control)

	self.pub = rospy.Publisher("/i2cmove", vision.msg.I2CMove)
	self.joint_pub = rospy.Publisher("/joints", JointState)

	i2cMessage.i2cthread = self

	self.thread = BlinkThread()
	self.thread.setDaemon(True)
	self.thread.start()


    def init_controls(self):
	with open(os.path.expanduser("~/control_map.json"), "r") as f:
	    self.controls = json.load(f)
	

    def on_joy(self, msg):
    	for name in self.controls.keys():
	    for part in self.controls[name].keys():
	        control = self.controls[name][part]
	        if "control" in control:
		    for condition in control["control"]:
		    	button_num = 99
			button_val = 0
			axis_num = 99
			axis_mul = 0
		    	if "button" in condition:
			    button_num = abs(condition["button"]) - 1
			    button_val = 0 if condition["button"] < 0 else 1
			if "axis" in condition:
			    axis_num = abs(condition["axis"]) - 1
			    axis_mul = -1 if condition["axis"] < 0 else 1
			if axis_num < 99 and (button_num == 99 or msg.buttons[button_num] == button_val):
			    # make sure the axis value is valid
			    # an invalid value can be used to indicate that no value is provided
			    # this is used to avoid different Joy message streams conflicting
			    if axis_num < len(msg.axes) and msg.axes[axis_num] >= -1.0 and msg.axes[axis_num] <= 1.0:
				control["joy"] = axis_mul * msg.axes[axis_num]

	# process special behaviors for button presses
	a = msg.axes
	if a[2] <= 1 or a[5] <= 1:  # variable button - front left/right bottom button
	   eye_lid(a[2], "right_eye")
           eye_lid(a[5], "left_eye")

	b = msg.buttons
	if b[0] == 1 and b[1] == 1:  # Red and Green Buttons
	    glaring()
	    time.sleep(1)
	    reseteye()
	    roll_eyes_sync(1)
	    glaring()
	    time.sleep(3)
	    blink(1)
	if b[0] == 1:  # Green Button
	    blink(1)
	if b[4] == 1:
	    blinkLeft(1)
	if b[5] == 1:
	    blinkRight(1)


    def joy_increment(self):
    	while True:
	    for name in self.controls.keys():
		for part in self.controls[name].keys():
		    control = self.controls[name][part]
		    if "joy" in control:
			if "type" in control and control["type"] == "rubber":
			    bounce = control["joy"]
			    bounce = (bounce * 0.5) + 0.5
			    move(name, part, bounce)
		    	else:
			    if "speed" in control:
				speed = control["speed"]
			    else:
				speed = 0.05
			    delta = control["joy"] * speed
			    if (delta != 0):
				if "pos" not in control:
				    control["pos"] = 0.5
				control["pos"] = max(0.0, min(control["pos"] + delta, 1.0))
				move(name, part, control["pos"])
			    elif "type" in control and control["type"] == "follow":
				# track time since last change in followed control
				follow = control["follow"]
				if "lastChange" not in follow:
				    follow["lastChange"] = time.time()
				else:
				    if "speed" in control:
					speed = control["speed"]
				    else:
					speed = 0.05
				    # 0 at start; 1 if >= delay
				    timeFactor = min(1.0, (time.time() - follow["lastChange"]) / follow["delaySec"])
				    delta = self.controls[follow["name"]][follow["part"]]["joy"] * speed * timeFactor**2
				    if (delta != 0):
					if "pos" not in control:
					    control["pos"] = 0.5
					control["pos"] = max(0.0, min(control["pos"] + delta, 1.0))
					rospy.loginfo('timeFactor=%g, delta=%g', timeFactor, delta)
					move(name, part, control["pos"])
				    else:
					del follow["lastChange"]

	    self.publish_joint_state()
	    time.sleep(0.05)


    def publish_joint_state(self):
	joint_state = JointState()
	joint_state.header.stamp = rospy.Time.now()

	for name in self.controls.keys():
	    for part in self.controls[name].keys():
		control = self.controls[name][part]
		if "pos" in control:
		    x = control["pos"]
		else:
		    x = 0.5
		# TODO fix this to allow each joint to interpret the angle of its limits
		x0 = 0.5
		angle = (x0 - x) * pi / 180.0
		joint_state.name.append(name + '.' + part)
		joint_state.position.append(angle)
		joint_state.velocity.append(0)

	self.joint_pub.publish(joint_state)


    def on_control(self, control):
	if control.data == "joy_control_reset":
	    rospy.loginfo('Received control message %s', control)
	    self.init_controls()


    def run(self):
    	t = threading.Thread(target=self.joy_increment)
	t.daemon = True
	t.start()
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


class BlinkThread(threading.Thread):
    def __init__(self):
    	threading.Thread.__init__(self)


    def run(self):
        while True:
	    time.sleep(randint(12, 15))
	    blink(1)


if __name__ == '__main__':
    try:
    	node = JoyControl()
	atexit.register(node.on_exit)
	node.run()
    except rospy.ROSInterruptException:
    	pass

