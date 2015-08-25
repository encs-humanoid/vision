#!/usr/bin/python
#===================================================================
# Print information about recognized faces
#
# Subscribes To:
# - /recognized_face
#
# Construct a histogram of the encounter ids received on the
# input topic and print it.
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import division
from vision.msg import RecognizedFace
import argparse
import atexit
import cv2
import numpy as np
import rospy
import sys


class Histogram(dict):
    def __init__(self):
    	pass

    def add(self, values):
    	for v in values:
	    if v not in self:
	    	self[v] = 0
	    self[v] += 1

    def display(self):
        bins = sorted(self.keys())
	cnts = [self[b] for b in bins]
	print " ".join(["%4.4s" % n for n in cnts])
	print " ".join(["%4.4s" % b for b in bins])


class SubFaceNode(object):
    def __init__(self):
	rospy.init_node('sub_face_node', anonymous=True)

	self.hist = Histogram()

	myargs = rospy.myargv(sys.argv) # process ROS args and return the rest
	#parser = argparse.ArgumentParser(description="Publish face messages given file names")
	#parser.add_argument("-f", "--facedb", dest="facedb", metavar="DIRECTORY", help="base directory to resolve file names to files")
	#self.options = parser.parse_args(myargs[1:])

	input_topic = rospy.get_param('~in', "/recognized_face")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~in'), input_topic)

	rospy.Subscriber(input_topic, RecognizedFace, self.on_recognized_face)


    def on_recognized_face(self, recognized_face):
    	self.hist.add(recognized_face.encounter_ids)
	self.hist.display()


    def run(self):
    	rospy.spin()
	#    rospy.loginfo('Failed to open image file ' + filename)


    def on_exit(self):
    	pass


if __name__ == '__main__':
    try:
	node = SubFaceNode()

	atexit.register(node.on_exit)
	node.run()
    except rospy.ROSInterruptException:
	pass

