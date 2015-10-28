#!/usr/bin/python
#===================================================================
# Publish a face from a file for testing face recognition.
#
# Publishes To:
# - /detected_face
# - /detected_face/image
#
# Given the path to a file in the face database, construct the
# DetectedFace message and publish to the detected_face topics.
# This can be used to test recognition.
#
# File names of faces to publish are read from stdin.
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import division
from vision.msg import DetectedFace
from cv_bridge import CvBridge, CvBridgeError
import argparse
import atexit
import cv2
import Image
import numpy as np
import rospy
import sensor_msgs.msg
import sys

class PubFaceNode(object):
    def __init__(self):
	rospy.init_node('pub_face_node', anonymous=True)

	self.bridge = CvBridge()

	myargs = rospy.myargv(sys.argv) # process ROS args and return the rest
	parser = argparse.ArgumentParser(description="Publish face messages given file names")
	parser.add_argument("-f", "--facedb", dest="facedb", default="facedb", metavar="DIRECTORY", help="base directory to resolve file names to files")
	self.options = parser.parse_args(myargs[1:])

	self.output_topic = rospy.get_param('~out', "/detected_face")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~out'), self.output_topic)

	self.face_pub = rospy.Publisher(self.output_topic, DetectedFace, queue_size=5)
	self.face_img_pub = rospy.Publisher(self.output_topic + "/image", sensor_msgs.msg.Image, queue_size=5)


    def run(self):
    	line = raw_input()
	while line:
	    filename = self.options.facedb + "/" + line.strip()
	    self.publish_face(filename)
	    line = raw_input()


    def publish_face(self, filename):
    	#try:
	db_image = Image.open(filename)
	if db_image:
	    cv_image = np.array(db_image, dtype=np.float32)
	    ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="32FC1")
	    detected_face = DetectedFace()
	    detected_face.x = 0
	    detected_face.y = 0
	    detected_face.w = 30
	    detected_face.h = 30
	    detected_face.left_eye_x = 1
	    detected_face.left_eye_y = 1
	    detected_face.left_eye_w = 12
	    detected_face.left_eye_h = 9
	    detected_face.right_eye_x = 18
	    detected_face.right_eye_y = 1
	    detected_face.right_eye_w = 12
	    detected_face.right_eye_h = 9
	    ros_image.header.seq = 0
	    ros_image.header.stamp = rospy.Time.now()
	    ros_image.header.frame_id = "test"
	    detected_face.image = ros_image
	    detected_face.header.stamp = rospy.Time.now()
	    detected_face.header.frame_id = "test"

	    self.face_pub.publish(detected_face)
	    self.face_img_pub.publish(detected_face.image)
	    rospy.loginfo('Published ' + filename + " to " + self.output_topic)
	#except:
	#    rospy.loginfo('Failed to open image file ' + filename)


    def on_exit(self):
    	pass


if __name__ == '__main__':
    try:
	node = PubFaceNode()

	atexit.register(node.on_exit)
	node.run()
    except rospy.ROSInterruptException:
	pass

