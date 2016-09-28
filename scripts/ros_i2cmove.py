#!/usr/bin/env python
# This ROS Node listens for move messages and sends the corresponding commands
# to the i2c bus.
#==================================================================================
from __future__ import division
import rospy
from vision.msg import I2CMove

class I2CMoveNode(object):
    def __init__(self):
	rospy.init_node('i2c_move_node')

	rospy.Subscriber("/i2cmove", I2CMove, self.on_move)

    def on_move(self, msg):
        print("Name: " + msg.name);
        print("Part: " + msg.part);
        print("Frac: " + str(msg.fraction));

    def run(self):
	rospy.spin()

if __name__ == '__main__':
    try:
    	node = I2CMoveNode()
	node.run()
    except rospy.ROSInterruptException:
    	pass

