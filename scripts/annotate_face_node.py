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
# - /face
# - /chessboard
# - /eye_alignment
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
# - face		topic name for receiving faces (raw)
# - chessboard		topic name for receiving chessboard points
# - align		topic name for receiving eye alignment
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
import geometry_msgs.msg
import numpy as np
#import oct2py
import pdb
import rospy
import sensor_msgs.msg
import std_msgs.msg
import sys
import time
import vision.msg
import vision_node


class Color(object):
    BLACK = (0, 0, 0)
    GRAY = (100, 100, 100)
    WHITE = (255, 255, 255)
    BLUE = (255, 0, 0)
    YELLOW = (0, 255, 255)
    GREEN = (0, 255, 0)
    RED = (0, 0, 255)


class AnnotateFaceNode(object):
    def __init__(self):
	rospy.init_node('annotate_face_node', anonymous=True)

	self.bridge = CvBridge()
	#self.oc = oct2py.Oct2Py()
	self.last_ros_image = None
	self.target_faces = []
	self.recognized_faces = []
	self.unrecognized_faces = []
	self.detected_faces = []
	self.faces = []
	self.last_chessboard = None
	self.last_chessboard_ts = time.time()
	self.eye_alignment = None

	#self.oc.load('face_model1.txt')

	myargs = rospy.myargv(sys.argv) # process ROS args and return the rest
	parser = argparse.ArgumentParser(description="Annotate faces in a ROS image stream")
	parser.add_argument("--debug", help="print out additional info", action="store_true")
	parser.add_argument("--nonn", help="disable the neural network recognizer", action="store_true")
	self.options = parser.parse_args(myargs[1:])

	self.frame_id = self.get_param('~frame', '/camera')
	image_topic = self.get_param('~image', "/camera/image_raw")
	target_topic = self.get_param('~target', "/target_face")
	recognized_topic = self.get_param('~recognized', "/recognized_face")
	unrecognized_topic = self.get_param('~unrecognized', "/unrecognized_face")
	detected_topic = self.get_param('~detected', "/detected_face")
	out_topic = self.get_param('~out', "/camera/image_annotated_raw")
	face_topic = self.get_param('~face', '/face')
	chessboard_topic = self.get_param('~chessboard', '/chessboard')
	align_topic = self.get_param('~align', '/eye_alignment')
	self.image_width = int(self.get_param('~image_width', '600'))
	self.image_height = int(self.get_param('~image_height', '400'))

	image_sub = rospy.Subscriber(image_topic, sensor_msgs.msg.Image, self.on_image)
	target_sub = rospy.Subscriber(target_topic, vision.msg.TargetFace, self.on_target_face)
	recognized_sub = rospy.Subscriber(recognized_topic, vision.msg.RecognizedFace, self.on_recognized_face)
	unrecognized_sub = rospy.Subscriber(unrecognized_topic, vision.msg.DetectedFace, self.on_unrecognized_face)
	detected_sub = rospy.Subscriber(detected_topic, vision.msg.DetectedFace, self.on_detected_face)
	face_sub = rospy.Subscriber(face_topic, vision.msg.Face, self.on_face)
	chessboard_sub = rospy.Subscriber(chessboard_topic, sensor_msgs.msg.PointCloud, self.on_chessboard)
	align_sub = rospy.Subscriber(align_topic, vision.msg.AlignEyes, self.on_eye_alignment)

	self.out_pub = rospy.Publisher(out_topic, sensor_msgs.msg.Image, queue_size=5)


    def get_param(self, param_name, param_default):
        value = rospy.get_param(param_name, param_default)
        rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param_name), value)
        return value


    def on_image(self, ros_image):
	self.last_ros_image = ros_image


    def on_target_face(self, target_face):
    	if target_face.header.frame_id == self.frame_id:
	    self.target_faces.append(target_face)


    def on_recognized_face(self, recognized_face):
    	if recognized_face.header.frame_id == self.frame_id:
	    self.recognized_faces.append(recognized_face)


    def on_unrecognized_face(self, unrecognized_face):
    	if unrecognized_face.header.frame_id == self.frame_id:
	    self.unrecognized_faces.append(unrecognized_face)


    def on_detected_face(self, detected_face):
	if detected_face.header.frame_id == self.frame_id:
	    self.detected_faces.append(detected_face)


    def on_face(self, face):
    	# store all the faces matching the node's frame_id
	if face.header.frame_id == self.frame_id:
	    self.faces.append(face)


    def on_chessboard(self, chessboard):
	#rospy.loginfo('Received chessboard for frame ' + chessboard.header.frame_id + ', my frame is ' + self.frame_id)
    	if chessboard.header.frame_id == self.frame_id:
	    self.last_chessboard = chessboard
	    self.last_chessboard_ts = time.time()


    def on_eye_alignment(self, align):
	tx = align.p.x
	ty = align.p.y
	th = align.p.z
	lcx = align.lc.x
	lcy = align.lc.y
	rcx = align.rc.x
	rcy = align.rc.y
    	self.eye_alignment = vision_node.StereoAlignment(self.image_height, [tx, ty], th, [lcx, lcy], [rcx, rcy])


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

	    # highlight eye overlaps in gray
	    if self.eye_alignment is not None:
		is_map_right_to_left = True if 'left' in self.frame_id else False
		# corners of rectangle in image coords
		w = self.image_width
		h = self.image_height
		points = np.array([[0, h], [0, 0], [w, 0], [w, h], [0, h]])
		mapped_points = self.eye_alignment.map_image_points(points, is_map_right_to_left)
		for i in range(len(mapped_points) - 1):
		    p0 = mapped_points[i]
		    p1 = mapped_points[i + 1]
		    # invert cartesian coords to image coords
		    cv2.line(color_image, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])), Color.GRAY)

	    now_sec = time.time()

	    # highlight raw faces in black
	    if len(self.faces) > 0:
	    	for f in self.faces:
		    cv2.rectangle(color_image, (f.x, f.y), (f.x + f.w, f.y + f.h), Color.BLACK, 2)
		# discard stored faces which are too old
		self.faces = [f for f in self.faces if now_sec - f.header.stamp.to_sec() < 0.1]

	    # highlight detected face in blue
	    if len(self.detected_faces) > 0:
	    	for d in self.detected_faces:
		    cv2.rectangle(color_image, (d.x, d.y), (d.x + d.w, d.y + d.h), Color.BLUE, 2)
		    if not self.options.nonn:
			#cv_image = self.bridge.imgmsg_to_cv2(d.image)
			#cw, ch = cv_image.shape[1::-1] # note image shape is h, w, d; reverse (h, w)->(w, h)
			#x = np.reshape(cv_image, (1, cw * ch))[0]
			#self.oc.push('x', x)
			pass
			#self.oc.eval('predictFace')
			#face_pred = self.oc.pull('face_pred')
			#cv2.putText(color_image, str(face_pred[0]), (d.x, d.y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, Color.WHITE)
			#print str(face_pred) + " at " + str(d.x) + ", " + str(d.y)
		self.detected_faces = [f for f in self.detected_faces if now_sec - f.header.stamp.to_sec() < 0.1]

	    # highlight unrecognized face in red
	    if len(self.unrecognized_faces) > 0:
	    	for u in self.unrecognized_faces:
		    cv2.rectangle(color_image, (u.x, u.y), (u.x + u.w, u.y + u.h), Color.RED, 2)
		self.unrecognized_faces = [f for f in self.unrecognized_faces if now_sec - f.header.stamp.to_sec() < 0.1]

	    # highlight recognized face in green
	    if len(self.recognized_faces) > 0:
	    	for r in self.recognized_faces:
		    cv2.rectangle(color_image, (r.x, r.y), (r.x + r.w, r.y + r.h), Color.GREEN, 2)
		# discard stored recognized faces which are too old
		self.recognized_faces = [f for f in self.recognized_faces if now_sec - f.header.stamp.to_sec() < 0.5]

	    # highlight target face in yellow
	    if len(self.target_faces) > 0:
	    	for t in self.target_faces:
		    if t.id == t.recog_id:
			cv2.rectangle(color_image, (t.x, t.y), (t.x + t.w, t.y + t.h), Color.YELLOW, 2)
			cv2.putText(color_image, str(t.name) + "-" + str(t.id), (t.x + 2, t.y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, Color.WHITE)
		    else:
			cv2.rectangle(color_image, (t.x, t.y), (t.x + t.w, t.y + t.h), Color.GREEN, 2)
			cv2.putText(color_image, str(t.recog_name) + "-" + str(t.recog_id), (t.x + 2, t.y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, Color.GREEN)
		self.target_faces = [f for f in self.target_faces if now_sec - f.header.stamp.to_sec() < 0.5]

	    # highlight chessboard points in yellow
	    if self.last_chessboard:
		p = self.last_chessboard.points
	    	for i in xrange(len(p) -1):
		    cv2.line(color_image, (int(p[i].x), int(p[i].y)), (int(p[i + 1].x), int(p[i + 1].y)), Color.YELLOW)
		if time.time() - self.last_chessboard_ts > 2:
		    self.last_chessboard = None

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

