#!/usr/bin/python

from __future__ import division
import rospy
import threading
import vision.msg
import time

class I2CMessageThread(threading.Thread):
    def __init__(self, node_name):
    	threading.Thread.__init__(self)
	rospy.init_node(node_name, anonymous=True)
	self.pub = rospy.Publisher("/i2cmove", vision.msg.I2CMove)

    def run(self):
	rospy.spin()

    def publish(self, name, part, fraction):
	move = vision.msg.I2CMove()
	move.name = name
	move.part = part
	move.fraction = fraction
    	self.pub.publish(move)

i2cthread = None

def move(name, part, fraction):
    global i2cthread
    if i2cthread is None:
    	i2cthread = I2CMessageThread(name + "_i2cnode")
	i2cthread.setDaemon(True)
	i2cthread.start()
	time.sleep(3)  # wait for node to get connected
    i2cthread.publish(name, part, fraction)
    rospy.loginfo("published i2c move for " + name + ", " + part + ", " + str(fraction))

