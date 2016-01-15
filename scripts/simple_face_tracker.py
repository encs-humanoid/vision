#!/usr/bin/python
#===================================================================
# This is the Simple Face Tracker.
#
# Subscribes To:
# - /stereo/left/image_raw
# - /stereo/right/image_raw
# - /control
#
# Publishes To:
# - /joy
#
# Detect faces in the input image streams and send joystick messages
# to center the face in the gaze.  Configure the center of gaze for
# each camera image.
#
# The input image topic must be in raw format.  This can be done
# by republishing the compressed image stream.  Example:
#
# rosrun image_transport republish compressed in:=/camera/image _image_transport:=compressed raw out:=/vision/image
#
# If using stereo cameras, republish each image stream to a separate raw stream, then
# run one face tracker on each stream.  Ensure that the gaze parameters are set correctly
# for the camera positions to avoid oscillations of the robot's head.
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
from math import pi
import numpy.random
import rospy
import sensor_msgs.msg
import std_msgs.msg
import sys
import time


class SimpleFaceTracker(object):
    def __init__(self):
	rospy.init_node("simple_face_tracker", anonymous=True)

	self.bridge = CvBridge()
	self.last_ros_image_left = None
	self.last_ros_image_right = None
	self.last_joint_state = None
	self.is_tracking = True
	self.time_of_last_face = 0
	self.look_straight_target_selected_time = 0

	myargs = rospy.myargv(sys.argv) # process ROS args and return the rest
	parser = argparse.ArgumentParser(description="Detect faces in a ROS image stream")
	parser.add_argument("--show", help="show image window", action="store_true")
	parser.add_argument("--debug", help="print out additional info", action="store_true")
	self.options = parser.parse_args(myargs[1:])

	left_topic = self.get_param("~left", "/stereo/left/image_raw")
	right_topic = self.get_param("~right", "/stereo/right/image_raw")
	control_topic = self.get_param("~control", "/control")
	joints_topic = self.get_param("~joints", "/joints")
	output_topic = self.get_param("~out", "/joy")

	# set the fraction of the image frame size where the a face appears
	# to be in the center of the camera's gaze.  Default is image center.
	self.center_gaze_left_x = float(self.get_param("~center_gaze_left_x", "0.5"))
	self.center_gaze_left_y = float(self.get_param("~center_gaze_left_y", "0.5"))
	self.center_gaze_right_x = float(self.get_param("~center_gaze_right_x", "0.5"))
	self.center_gaze_right_y = float(self.get_param("~center_gaze_right_y", "0.5"))

	self.max_no_face_staring_time_sec = float(self.get_param("~max_no_face_staring_time_sec", "2.0"))
	# maximum seconds to wait for target to be reached before selecting a new target
	self.max_target_time_sec = float(self.get_param("~max_target_time_sec", "10.0"))

	#  target achieved threshold in radians (parameter specified in degrees)
	self.pose_target_achieved_rad = (pi / 180.0) * float(self.get_param("~pose_target_achieved_deg", "2.0"))
	# max time to hold the target pose
	self.pose_target_hold_sec = float(self.get_param("~pose_target_hold_sec", "2.0"))
	self.pose_target_achieved_time = 0  # the time the target was achieved - used to determine the hold time
	self.tilt_joint = self.get_param("~tilt_joint", "torso_neck_joint")
	self.pan_joint = self.get_param("~pan_joint", "upper_neck_head_joint")

	self.pose_target = {self.pan_joint: 0.0, self.tilt_joint: 0.0}
	self.pose = self.pose_target

	# set the gain multiplier which converts the fractional position of
	# a face with respect to the center of gaze into a joystick analog
	# signal.  Joystick signals should be in [-1, 1], so gain is in [0, 1]
	self.gain = float(self.get_param("~gain", "0.75"))

	face_cascade_path = self.get_param("~face_cascade", "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml")

	self.face_cascade = cv2.CascadeClassifier(face_cascade_path)
	left_image_sub = rospy.Subscriber(left_topic, sensor_msgs.msg.Image, self.on_left_image)
	right_image_sub = rospy.Subscriber(right_topic, sensor_msgs.msg.Image, self.on_right_image)
	control_sub = rospy.Subscriber(control_topic, std_msgs.msg.String, self.on_control)
	joints_sub = rospy.Subscriber(joints_topic, sensor_msgs.msg.JointState, self.on_joints)
	self.joy_pub = rospy.Publisher(output_topic, sensor_msgs.msg.Joy, queue_size=1)


    def get_param(self, param_name, param_default):
        value = rospy.get_param(param_name, param_default)
        rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param_name), value)
        return value


    def on_left_image(self, ros_image):
	self.last_ros_image_left = ros_image


    def on_right_image(self, ros_image):
    	self.last_ros_image_right = ros_image


    def on_control(self, control):
        rospy.loginfo('Received control message %s', control)
	if control.data == "resume_face_tracking":
	    self.is_tracking = True
	elif control.data == "stop_face_tracking":
	    self.is_tracking = False


    def on_joints(self, joint_state):
	pose = {}
	for i in range(len(joint_state.name)):
	    pose[joint_state.name[i]] = joint_state.position[i]
	# assign to self.pose in a single statement to avoid multithreading issue
	# (KeyError) with an incomplete dict
	self.pose = pose
    	self.last_joint_state = joint_state


    def run(self):
	rate = rospy.Rate(30) # 30 Hz

	try:
	    while not rospy.is_shutdown():
		rate.sleep() # give ROS a chance to run
		# process left image
		if self.last_ros_image_left:
		    # grab the last ROS image we have received and clear the
		    # image handle so we don't pick up the same image next time
		    ros_image, self.last_ros_image_left = self.last_ros_image_left, None

		    if self.is_tracking:
			self.process_image(ros_image, self.center_gaze_left_x, self.center_gaze_left_y)

		# process right image
		if self.last_ros_image_right:
		    ros_image, self.last_ros_image_right = self.last_ros_image_right, None
		    if self.is_tracking:
			self.process_image(ros_image, self.center_gaze_right_x, self.center_gaze_right_y)

		# if no face in view, look straight
		if not self.is_face_in_view() and self.is_tracking:
		    self.look_around()
	except KeyboardInterrupt:
	    pass

	self.on_exit()


    def is_face_in_view(self):
    	return time.time() - self.time_of_last_face <= self.max_no_face_staring_time_sec


    def on_exit(self):
	if self.options.show:
	    cv2.destroyAllWindows()


    def process_image(self, ros_image, center_gaze_x, center_gaze_y):
	try:
	    # convert the ROS image to OpenCV and convert it to gray scale
	    color_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
	    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

	    faces = self.find_faces(gray_image)

	    # Face Tracking - generate Joy messages to move face toward the center of gaze
	    if len(faces) > 0:
		self.time_of_last_face = time.time()
		(x, y, w, h) = faces[0]
		if self.options.show:
		    # Draw a rectangle around the face
		    cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 0, 255), 2)
		iw, ih = color_image.shape[1::-1] # shape is h, w, d; reverse (h, w)->(w, h)
		fx, fy = (x + w/2, y + h/2); # center of face rectangle
		jx = self.gain * (iw * center_gaze_x - fx) / iw
		jy = self.gain * (ih * center_gaze_y - fy) / ih
		if self.options.debug:
		    print str((fx, fy)) + ": w=" + str(w) + ", h=" + str(h) + ", jx=" + str(jx) + ", jy=" + str(jy)

		self.joy_pub.publish(self.new_joy_message(jx, jy))
#		print "tracking face at " + str(fx) + ", " + str(fy)
	    elif self.is_face_in_view():
		self.joy_pub.publish(self.new_joy_message(0, 0))

	    # TODO fix the show option to work with two image streams
	    # consider drawing the left and right images side-by-side in the same window
	    if self.options.show:
		cv2.imshow('Video', color_image)

		# Let OpenCV's event loop run so the video will display
		cv2.waitKey(1)
	except CvBridgeError, e:
	    print e


    def find_faces(self, gray_image):
	faces = self.face_cascade.detectMultiScale(
	    gray_image,
	    scaleFactor=1.2,
	    minNeighbors=5,
	    minSize=(30, 30),
	    flags=cv2.cv.CV_HAAR_SCALE_IMAGE
	)
	return faces


    def new_joy_message(self, x_axis, y_axis):
	joy = sensor_msgs.msg.Joy()
	joy.axes = [x_axis, y_axis, 0.0, 0.0, 0.0, 0.0, 0.0]
	joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	return joy


    def look_around(self):
    	'''
	Cause the robot to look around, rather than remaining stuck in a static pose.

	To do this, select a target pose.
	Determine the direction vector from the current pose to the target pose and send a joystick
	message to move the head in the desired direction.
	If sufficient time has passed since the selection of the previous target pose, or the
	current pose is near enough the target and the target pose hold time has elapsed, then
	select a new target pose and repeat.
	'''
	# check if we have spent enough time on this target pose
	now = time.time()
	if now - self.look_straight_target_selected_time >= self.max_target_time_sec:
	    rospy.loginfo('Timed out attempting target pose')
	    self.select_look_around_target()
	# check if the target pose has been held for sufficient time
	elif self.pose_target_achieved_time > 0 and now - self.pose_target_achieved_time >= self.pose_target_hold_sec:
	    rospy.loginfo('Target pose held until ' + str(now))
	    self.select_look_around_target()
	    self.pose_target_achieved_time = 0
	# check if the target pose has been achieved, and store the time
	elif self.max_pose_deviation_from_target_rad() <= self.pose_target_achieved_rad:
	    if self.pose_target_achieved_time == 0:
		rospy.loginfo('Target pose achieved at ' + str(now))
		self.pose_target_achieved_time = now
	
	# calculate the direction to the target pose and send a joystick message
	iw, ih = pi, pi   # no particular meaning to these, but they work empirically
	fx, fy = self.pose[self.pan_joint], self.pose[self.tilt_joint]
	jx = self.gain * (iw * self.pose_target[self.pan_joint] - fx) / iw
	jy = 3 * self.gain * (ih * self.pose_target[self.tilt_joint] - fy) / ih
	#print "look_around " + str(jx) + ", " + str(jy) + ", pose=" + str(self.pose)
	self.joy_pub.publish(self.new_joy_message(jx, jy))


    def select_look_around_target(self):
   	# select new random target from a 2D normal distribution centered at (0,0)
	# covariance selected to keep pan joint in [-0.15, 0.15] and tilt joint in [-0.05, 0.05]
	# which were determined empirically to be satisfactory values
	# TODO make these values part of the robot instance configuration
	# 3 stdev
	# 0.0025 = (0.15 / 3) ** 2
	# 0.000256 = (0.048 / 3) ** 2
	# 2 stdev
	# 0.005625 = (0.15 / 2) ** 2
	# 0.000625 = (0.05 / 2) ** 2
	t = numpy.random.multivariate_normal([0, 0], [[0.005625, 0], [0, 0.000625]])
	t[0] = min(max(t[0], -0.15), 0.15)
	t[1] = min(max(t[1], -0.05), 0.05)
	# TEMP read pose from file
	#with open('target_pose.txt', 'r') as f:
	#    lines = f.readlines()
	#t = [float(v) for v in lines[0].split()]
	self.pose_target = {self.pan_joint: t[0], self.tilt_joint: t[1]}
        rospy.loginfo('Target pose set to ' + str(self.pose_target))
	self.look_straight_target_selected_time = time.time()


    def max_pose_deviation_from_target_rad(self):
    	'''
	Calculate the maximum deviation between the target pose and the current pose in radians.
	'''
	if self.last_joint_state is None:
	    return 0.0

	pan_deviation = abs(self.pose_target[self.pan_joint] - self.pose[self.pan_joint])
	tilt_deviation = abs(self.pose_target[self.pan_joint] - self.pose[self.tilt_joint])
	return max(pan_deviation, tilt_deviation)


if __name__ == '__main__':
    try:
	node = SimpleFaceTracker()

	atexit.register(node.on_exit)
	node.run()
    except rospy.ROSInterruptException:
	pass


