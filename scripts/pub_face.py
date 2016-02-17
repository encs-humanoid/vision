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
from vision.msg import RecognizedFace
from vision.msg import TargetFace
from cv_bridge import CvBridge, CvBridgeError
import argparse
import atexit
import cv2
import facedb
import Image
import numpy as np
import os
import random
import rospy
import sensor_msgs.msg
import sys
import time

class PubFaceNode(object):
    def __init__(self, output_topic, recog_topic, unrec_topic, options=None):
	rospy.init_node('pub_face_node', anonymous=True)

	self.output_topic = output_topic
	self.recog_topic = recog_topic
	self.unrec_topic = unrec_topic

	self.options = options

	self.bridge = CvBridge()
	self.received_recog_face = False
	self.received_unrec_face = False
	self.recognized_face = None
	self.unrecognized_face = None

	#rospy.Subscriber(self.recog_topic, TargetFace, self.on_recognized_face)
	if "target" in self.recog_topic:
	    rospy.Subscriber(self.recog_topic, TargetFace, self.on_recognized_face)
	else:
	    rospy.Subscriber(self.recog_topic, RecognizedFace, self.on_recognized_face)
	rospy.Subscriber(self.unrec_topic, DetectedFace, self.on_unrecognized_face)
	self.face_pub = rospy.Publisher(self.output_topic, DetectedFace, queue_size=5)
	self.face_img_pub = rospy.Publisher(self.output_topic + "/image", sensor_msgs.msg.Image, queue_size=5)
    	self.rate = rospy.Rate(100)  # 100 Hz


    def on_recognized_face(self, recognized_face):
    	print 'received recognized face'
    	self.received_recog_face = True
	#self.recognized_id = target_face.recog_id
	self.recognized_face = recognized_face


    def on_unrecognized_face(self, detected_face):
    	print 'received unrecognized face'
    	self.received_unrec_face = True
	self.unrecognized_face = detected_face
	#self.recognized_id = None


    def run(self):
    	if self.options.all:
	    self.rate.sleep()
	    time.sleep(3)  # allow time for subscribers to connect
	    self.rate.sleep()
	    fdb = facedb.FaceDB(self.options.facedb)
	    for encounter in fdb.iterate_unique():
	    	# generate random coordinates for the encounter
		x = random.randrange(0, 570)
		y = random.randrange(0, 370)
		for face in encounter.iterate():
		    self.publish_face(face.image_file, x, y)
		    # wait for recognition response before sending next face
		    # to avoid faces being dropped by the recognize face node
		    self.wait_for_recognition()
		    #if self.received_recog_face:
			#rospy.loginfo('Recognized ' + face.image_file + " as " + str(self.recognized_id))
	else:
	    line = raw_input()
	    while line:
		filename = self.options.facedb + "/" + line.strip()
		if not os.path.exists(filename):
		    filename = line.strip()
		self.publish_face(filename)
		line = raw_input()


    def publish_face(self, filename, x=0, y=0):
    	#try:
	db_image = Image.open(filename)
	if db_image:
	    cv_image = np.array(db_image, dtype=np.float32)
	    ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="32FC1")
	    detected_face = DetectedFace()
	    detected_face.x = x
	    detected_face.y = y
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


    def wait_for_recognition(self):
	self.received_recog_face = False
	self.received_unrec_face = False
	while not rospy.is_shutdown():
	    if self.received_recog_face or self.received_unrec_face:
		break
	    self.rate.sleep()
	if rospy.is_shutdown():
	    raise rospy.ROSInterruptException()


    def on_exit(self):
    	print 'exiting'
    	sys.exit(1)


if __name__ == '__main__':
    try:
	myargs = rospy.myargv(sys.argv) # process ROS args and return the rest
	parser = argparse.ArgumentParser(description="Publish face messages given file names")
	parser.add_argument("-f", "--facedb", dest="facedb", default="facedb", metavar="DIRECTORY", help="base directory to resolve file names to files")
	parser.add_argument("--all", help="publish all faces in facedb", action="store_true")
	options = parser.parse_args(myargs[1:])

	output_topic = rospy.get_param('~out', '/detected_face')
	recog_topic = rospy.get_param('~recog', '/target_face')
	unrec_topic = rospy.get_param('~unrec', '/unrecognized_face')
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~out'), output_topic)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~recog'), recog_topic)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~unrec'), unrec_topic)

	node = PubFaceNode(output_topic, recog_topic, unrec_topic, options)
	atexit.register(node.on_exit)
	node.run()
    except rospy.ROSInterruptException:
	sys.exit(1)

