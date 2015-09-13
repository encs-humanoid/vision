#!/usr/bin/python
#===================================================================
# This node annotates the image stream with information about recognized faces
#
# Subscribes To:
# - /camera/image_raw
# - /target_face
# - /recognized_face
# - /unrecognized_face
# - /detected_face
#
# Publishes To:
# - /camera/image_annotated_raw
#
# Parameters:
# - image		topic name for the input video stream
# - target		topic name for receiving target faces
# - recognized		topic name for receiving recognized faces
# - unrecognized	topic name for receiving unrecognized faces
# - detected		topic name for receiving detected faces
# - out			topic name for the output, annotated video stream
#
# Subscribe to a video topic and republish the images with annotations showing
# information collected from other topics about detected faces, recognized faces,
# and the target face.
# 
# The input image topic must be in raw format.  This can be done
# by republishing the compressed image stream.  Example:
#
# rosrun image_transport republish compressed in:=/camera/image _image_transport:=compressed raw out:=/vision/image
#
# If using stereo cameras, republish each image stream to a separate raw stream, then
# run one face annotator node on each stream.
#
# rosrun image_transport republish compressed in:=/stereo/left/camera/image _image_transport:=compressed raw out:=/stereo/left/image_raw
# rosrun image_transport republish compressed in:=/stereo/right/camera/image _image_transport:=compressed raw out:=/stereo/right/image_raw
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import division
from cv_bridge import CvBridge, CvBridgeError
import argparse
import atexit
import cv2
import face_util
import numpy as np
#import oct2py
import pdb
import rospy
import sensor_msgs.msg
import std_msgs.msg
import sys
import time
import vision.msg


class Color(object):
    BLUE = (255, 0, 0)
    YELLOW = (255, 255, 0)
    GREEN = (0, 255, 0)
    RED = (0, 0, 255)


class AnnotateFaceNode(object):
    def __init__(self):
	rospy.init_node('annotate_face_node', anonymous=True)

	self.bridge = CvBridge()
	#self.oc = oct2py.Oct2Py()
	self.last_ros_image = None
	self.last_target_face = None
	self.last_recognized_face = None
	self.last_unrecognized_face = None
	self.last_detected_face = None
	self.last_detected_face_ts = time.time()

	#self.oc.load('face_model1.txt')

	myargs = rospy.myargv(sys.argv) # process ROS args and return the rest
	parser = argparse.ArgumentParser(description="Annotate faces in a ROS image stream")
	parser.add_argument("--debug", help="print out additional info", action="store_true")
	self.options = parser.parse_args(myargs[1:])

	image_topic = self.get_param('~image', "/camera/image_raw")
	target_topic = self.get_param('~target', "/target_face")
	recognized_topic = self.get_param('~recognized', "/recognized_face")
	unrecognized_topic = self.get_param('~unrecognized', "/unrecognized_face")
	detected_topic = self.get_param('~detected', "/detected_face")
	out_topic = self.get_param('~out', "/camera/image_annotated_raw")

	image_sub = rospy.Subscriber(image_topic, sensor_msgs.msg.Image, self.on_image)
	target_sub = rospy.Subscriber(target_topic, vision.msg.TargetFace, self.on_target_face)
	recognized_sub = rospy.Subscriber(recognized_topic, vision.msg.RecognizedFace, self.on_recognized_face)
	unrecognized_sub = rospy.Subscriber(unrecognized_topic, vision.msg.DetectedFace, self.on_unrecognized_face)
	detected_sub = rospy.Subscriber(detected_topic, vision.msg.DetectedFace, self.on_detected_face)
	self.out_pub = rospy.Publisher(out_topic, sensor_msgs.msg.Image, queue_size=5)


    def get_param(self, param_name, param_default):
        value = rospy.get_param(param_name, param_default)
        rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param_name), value)
        return value


    def on_image(self, ros_image):
	self.last_ros_image = ros_image


    def on_target_face(self, target_face):
	self.last_target_face = target_face
        rospy.loginfo('Received target face message')


    def on_recognized_face(self, recognized_face):
	# TODO support multiple faces in frame
	self.last_recognized_face = recognized_face
	rospy.loginfo('Received recognized face message')


    def on_unrecognized_face(self, unrecognized_face):
	# TODO support multiple faces in frame
	self.last_unrecognized_face = unrecognized_face
	rospy.loginfo('Received unrecognized face message')


    def on_detected_face(self, detected_face):
	# TODO support multiple faces in frame
	self.last_detected_face = detected_face
	self.last_detected_face_ts = time.time()
	print detected_face.header.stamp, detected_face.header.frame_id, detected_face.x, detected_face.y, detected_face.w, detected_face.h
	#rospy.loginfo('Received detected face message')


    def run(self):
	rate = rospy.Rate(15) # 15 Hz

	try:
	    while not rospy.is_shutdown():
		rate.sleep() # give ROS a chance to run
		if self.last_ros_image:
		    # grab the last ROS image we have received and clear the
		    # image handle so we don't pick up the same image next time
		    ros_image, self.last_ros_image = self.last_ros_image, None

		    self.process_image(ros_image)
	except KeyboardInterrupt:
	    pass

	self.on_exit()


    def on_exit(self):
	#self.oc.exit()
	pass


    def process_image(self, ros_image):
	"""
	Annotate and republish the image frame.
	"""
	try:
	    # convert the ROS image to OpenCV image
	    color_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")

	    # TODO correlate the timestamps of messages to the image timestamp

	    # TODO filter by frame id e.g., recognized_face.header.frame_id

	    # annotate the image
	    # highlight detected face in blue
	    d = self.last_detected_face
	    if time.time() - self.last_detected_face_ts > 2:
	    	self.last_detected_face = None
	    if d:
		cv2.rectangle(color_image, (d.x, d.y), (d.x + d.w, d.y + d.h), Color.BLUE, 2)
		#cv_image = self.bridge.imgmsg_to_cv2(d.image)
		#cw, ch = cv_image.shape[1::-1] # note image shape is h, w, d; reverse (h, w)->(w, h)
		#x = np.reshape(cv_image, (1, cw * ch))[0]
		#self.oc.push('x', x)
		pass
		#self.oc.eval('predictFace')
		#face_pred = self.oc.pull('face_pred')
		#cv2.putText(color_image, str(face_pred[0]), (d.x, d.y - 10), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
		#print str(face_pred) + " at " + str(d.x) + ", " + str(d.y)

	    # highlight unrecognized face in red
	    u, self.last_unrecognized_face = self.last_unrecognized_face, None
	    if u:
		cv2.rectangle(color_image, (u.x, u.y), (u.x + u.w, u.y + u.h), Color.RED, 2)

	    # highlight recognized face in green
	    r, self.last_recognized_face = self.last_recognized_face, None
	    if r:
		cv2.rectangle(color_image, (r.x, r.y), (r.x + r.w, r.y + r.h), Color.GREEN, 2)

	    # TODO highlight target face in yellow
	    t, self.last_target_face = self.last_target_face, None

	    # republish the image to the output topic
	    self.out_pub.publish(self.bridge.cv2_to_imgmsg(color_image, "bgr8"))
	except CvBridgeError, e:
	    print e


if __name__ == '__main__':
    try:
	node = AnnotateFaceNode()

	atexit.register(node.on_exit)
	node.run()
    except rospy.ROSInterruptException:
	pass

