#!/usr/bin/python
#===================================================================
# This is the Simple Face Tracker.
#
# Subscribes To:
# - /stereo/left/image_raw
# - /stereo/right/image_raw
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
import rospy
import sensor_msgs.msg
import sys


class SimpleFaceTracker(object):
    def __init__(self):
	rospy.init_node('simple_face_tracker', anonymous=True)

	self.bridge = CvBridge()
	self.last_ros_image = None

	myargs = rospy.myargv(sys.argv) # process ROS args and return the rest
	parser = argparse.ArgumentParser(description="Detect faces in a ROS image stream")
	parser.add_argument("--show", help="show image window", action="store_true")
	parser.add_argument("--debug", help="print out additional info", action="store_true")
	self.options = parser.parse_args(myargs[1:])

	input_topic = self.get_param('~in', "/vision/image")
	output_topic = self.get_param('~out', "/joy")

	# set the fraction of the image frame size where the a face appears
	# to be in the center of the camera's gaze.  Default is image center.
	self.center_gaze_x = float(self.get_param('~center_gaze_x', "0.5"))
	self.center_gaze_y = float(self.get_param('~center_gaze_y', "0.5"))

	# set the gain multiplier which converts the fractional position of
	# a face with respect to the center of gaze into a joystick analog
	# signal.  Joystick signals should be in [-1, 1], so gain is in [0, 1]
	self.gain = float(self.get_param('~gain', "0.75"))

	face_cascade_path = self.get_param('~face_cascade', '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml')

	self.face_cascade = cv2.CascadeClassifier(face_cascade_path)
	image_sub = rospy.Subscriber(input_topic, sensor_msgs.msg.Image, self.on_image)
	self.joy_pub = rospy.Publisher(output_topic, sensor_msgs.msg.Joy, queue_size=1)


    def get_param(self, param_name, param_default):
        value = rospy.get_param(param_name, param_default)
        rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param_name), value)
        return value


    def on_image(self, ros_image):
	self.last_ros_image = ros_image


    def run(self):
	rate = rospy.Rate(30) # 30 Hz

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
	if self.options.show:
	    cv2.destroyAllWindows()


    def process_image(self, ros_image):
	try:
	    # convert the ROS image to OpenCV and convert it to gray scale
	    color_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
	    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

	    faces = self.find_faces(gray_image)

	    # Face Tracking - generate Joy messages to move face toward the center of gaze
	    if len(faces) > 0:
		(x, y, w, h) = faces[0]
		if self.options.show:
		    # Draw a rectangle around the face
		    cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 0, 255), 2)
		iw, ih = color_image.shape[1::-1] # shape is h, w, d; reverse (h, w)->(w, h)
		fx, fy = (x + w/2, y + h/2); # center of face rectangle
		jx = self.gain * (iw * self.center_gaze_x - fx) / iw
		jy = self.gain * (ih * self.center_gaze_y - fy) / ih
		if self.options.debug:
		    print str((fx, fy)) + ": w=" + str(w) + ", h=" + str(h) + ", jx=" + str(jx) + ", jy=" + str(jy)

		self.joy_pub.publish(self.new_joy_message(jx, jy))
	    else:
		self.joy_pub.publish(self.new_joy_message(0, 0))

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


if __name__ == '__main__':
    try:
	node = SimpleFaceTracker()

	atexit.register(node.on_exit)
	node.run()
    except rospy.ROSInterruptException:
	pass


