#!/usr/bin/env python
# This ROS Node listens for move messages and sends the corresponding commands
# to the i2c bus.
#==================================================================================
from __future__ import division
from i2ctest import move, reload
import rospy
from vision.msg import I2CMove
from sensor_msgs.msg  import Joy
import std_msgs.msg
import threading
import time

class I2CMoveNode(object):
    def __init__(self):
	rospy.init_node('i2c_move_node')
	
	self.arm_lift_speed = 0.05
	self.arm_pivot_speed = 0.05
	self.right_arm_lift = 0.0
	self.right_arm_pivot = 0.6
	self.currentX = 0.0
	self.currentY = 0.0

	# neck controls
	self.neck_pan_speed = 0.025
	self.neck_tilt_speed = 0.05
	self.neck_roll_speed = 0.05
	self.neck_pan_position = 0.5
	self.neck_tilt_position = 0.5
	self.neck_roll_position = 0.5
	self.currentHX = 0.0
	self.currentHY = 0.0
	self.currentHZ = 0.0

	rospy.Subscriber("/i2cmove", I2CMove, self.on_move)
	# remove joy message processing in favor of joy_control node doing it
	#rospy.Subscriber("/joy", Joy, self.on_joy)
	rospy.Subscriber("/control", std_msgs.msg.String, self.on_control)

    def on_move(self, msg):
        print("Name: " + msg.name)
        print("Part: " + msg.part)
        print("Frac: " + str(msg.fraction))
	move(msg.name, msg.part, msg.fraction)
	
    def on_joy(self, msg):
    	self.currentX = -msg.axes[3]
	self.currentY = msg.axes[2]
	if msg.buttons[2] == 1:  # X button
	    self.currentHZ = msg.axes[4]
	else:
	    self.currentHX = -msg.axes[4]
	    self.currentHZ = 0.0
	self.currentHY = -msg.axes[5]

    def joy_increment(self):
    	deltaX = 0.0
	deltaY = 0.0 
   	while True:
	    deltaX = self.currentX * self.arm_lift_speed
	    deltaY = self.currentY * self.arm_pivot_speed
	    if (deltaX != 0):
		if (self.currentX < 0.25 or self.currentX > -0.25):
		    self.right_arm_lift += deltaX
		    self.right_arm_lift = max(0.0, min(self.right_arm_lift, 1.0))
		    move("right_arm", "lift", self.right_arm_lift)
	    if (deltaY != 0):
	    	if (self.currentY < 0.25 or self.currentY > -0.25):
		    self.right_arm_pivot += deltaY
		    self.right_arm_pivot = max(0.0, min(self.right_arm_pivot, 1.0))
		    move("right_arm", "piv", self.right_arm_pivot)
	    deltaHX = self.currentHX * self.neck_pan_speed
	    if deltaHX != 0:
		self.neck_pan_position = max(0.0, min(self.neck_pan_position + deltaHX, 1.0))
		move("neck", "pan", self.neck_pan_position)
	    deltaHY = self.currentHY * self.neck_tilt_speed
	    if deltaHY != 0:
	    	self.neck_tilt_position = max(0.0, min(self.neck_tilt_position + deltaHY, 1.0))
		move("neck", "tilt", self.neck_tilt_position)
	    deltaHZ = self.currentHZ * self.neck_roll_speed
	    if deltaHZ != 0:
	    	self.neck_roll_position = max(0.0, min(self.neck_roll_position + deltaHZ, 1.0))
		move("neck", "roll", self.neck_roll_position)
	    time.sleep(0.05)

    def on_control(self, control):
        rospy.loginfo('Received control message %s', control)
	if control.data == "i2cmove_reload":
	    reload()

    def run(self):
    	t = threading.Thread(target=self.joy_increment)
	t.daemon = True
	t.start()
	rospy.spin()

if __name__ == '__main__':
    try:
    	node = I2CMoveNode()
	node.run()
    except rospy.ROSInterruptException:
    	pass

