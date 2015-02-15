#!/usr/bin/python
#===================================================================
# This is the Face Detect Node.
#
# Subscribes To:
# - /vision/image
#
# Publishes To:
# - /detected_face
# - /detected_face/image
#
# Detect faces with two eyes in the incoming image stream, crop
# and normalize them, and publish the segmented images with face
# and eye locations.
#
# The input image topic must be in raw format.  This can be done
# by republishing the compressed image stream.  Example:
#
# rosrun image_transport republish compressed in:=/camera/image _image_transport:=compressed raw out:=/vision/image
# rosrun image_transport republish compressed in:=/stereo/left/camera/image _image_transport:=compressed raw out:=/vision/image
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import division
from align_face import CropFace
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

face_cascade_path = '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml'
eye_cascade_path = '/usr/share/opencv/haarcascades/haarcascade_eye.xml'

class FaceDetectNode(object):
    def __init__(self):
	rospy.init_node('face_detect_node', anonymous=True)

	self.bridge = CvBridge()
	self.last_ros_image = None
	self.face_cascade = cv2.CascadeClassifier(face_cascade_path)
	self.eye_cascade = cv2.CascadeClassifier(eye_cascade_path)

	myargs = rospy.myargv(sys.argv) # process ROS args and return the rest
	parser = argparse.ArgumentParser(description="Detect faces in a ROS image stream")
	parser.add_argument("-o", "--output", dest="output_directory", metavar="DIRECTORY", help="write cropped faces to directory")
	parser.add_argument("--show", help="show image window", action="store_true")
	self.options = parser.parse_args(myargs[1:])

	input_topic = rospy.get_param('~in', "/vision/image")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~in'), input_topic)
	output_topic = rospy.get_param('~out', "/detected_face")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~out'), output_topic)

	image_sub = rospy.Subscriber(input_topic, sensor_msgs.msg.Image, self.on_image)
	self.face_pub = rospy.Publisher(output_topic, DetectedFace, queue_size=5)
	self.face_img_pub = rospy.Publisher(output_topic + "/image", sensor_msgs.msg.Image, queue_size=5)


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

	    i = 0
	    for (x, y, w, h) in faces:
		# DEBUG Draw a rectangle around the faces
		if self.options.show:
		    cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 0, 255), 2)

		# Extract just the face as a subimage
		face_only_gray = gray_image[y:y+h, x:x+w]

		# look for eyes only in the top of the image
		eye_band_upper=0.2
		eye_band_lower=0.6
		eyes = self.find_eyes(face_only_gray[eye_band_upper*h:eye_band_lower*h, :])

		# since eyes are found in a band, shift the coordinates to be relative to the face
		eyes = [(ex, ey + int(eye_band_upper*h), ew, eh) for (ex, ey, ew, eh) in eyes]

		# DEBUG Draw rectangles around the eyes
		if self.options.show:
		    for (ex, ey, ew, eh) in eyes:
			cv2.rectangle(color_image, (x+ex, y+ey), (x+ex+ew, y+ey+eh), (0, 255, 0), 2)

		if len(eyes) == 2:
		    cv_crop, left_eye, right_eye = crop_and_normalize(face_only_gray, eyes)
		    
		    detected_face = self.create_detected_face_msg(x, y, w, h, left_eye, right_eye, cv_crop, ros_image)
	
		    self.face_pub.publish(detected_face)

		    face_image = detected_face.image
		    face_image.header.stamp = rospy.Time.now()
		    self.face_img_pub.publish(face_image)

		    if self.options.output_directory:
			name = str(ros_image.header.stamp) + "_crop" + str(i)
			cv2.imwrite(self.options.output_directory + "/" + name + ".png", cv_crop)

		    i += 1

	    if self.options.show:
		cv2.imshow('Video', color_image)

		# Let OpenCV's event loop run so the video will display
		cv2.waitKey(1)
	except CvBridgeError, e:
	    print e


    def create_detected_face_msg(self, x, y, w, h, left_eye, right_eye, cv_crop, ros_image):
	detected_face = DetectedFace()
	detected_face.x = x
	detected_face.y = y
	detected_face.w = w
	detected_face.h = h
	detected_face.left_eye_x = left_eye[0]
	detected_face.left_eye_y = left_eye[1]
	detected_face.left_eye_w = left_eye[2]
	detected_face.left_eye_h = left_eye[3]
	detected_face.right_eye_x = right_eye[0]
	detected_face.right_eye_y = right_eye[1]
	detected_face.right_eye_w = right_eye[2]
	detected_face.right_eye_h = right_eye[3]
	face_image = self.bridge.cv2_to_imgmsg(cv_crop, encoding="32FC1")
	# copy the header info from the original image to the cutout face image
	face_image.header.seq = ros_image.header.seq
	face_image.header.stamp = ros_image.header.stamp
	face_image.header.frame_id = ros_image.header.frame_id
	detected_face.image = face_image
	detected_face.header.stamp = rospy.Time.now()
	detected_face.header.frame_id = ros_image.header.frame_id
	return detected_face


    def find_faces(self, gray_image):
	faces = self.face_cascade.detectMultiScale(
	    gray_image,
	    scaleFactor=1.2,
	    minNeighbors=5,
	    minSize=(30, 30),
	    flags=cv2.cv.CV_HAAR_SCALE_IMAGE
	)
	return faces


    def find_eyes(self, face_only_gray):
	eyes = self.eye_cascade.detectMultiScale(face_only_gray,
		    scaleFactor=1.1,
		    minNeighbors=5,
		    minSize=(5, 5),
		    flags=cv2.cv.CV_HAAR_SCALE_IMAGE
	)
	return eyes


def crop_and_normalize(face_only_gray, eyes, target_intensity=95, target_range=150, offset_pct=(0.2,0.2), dest_sz=(30,30)):
    left_eye = min(eyes)
    right_eye = max(eyes)
    el_center = get_center(left_eye)
    er_center = get_center(right_eye)

    # Align eyes and crop image
    image = Image.fromarray(face_only_gray)
    crop = CropFace(image, eye_left=el_center, eye_right=er_center, offset_pct=offset_pct, dest_sz=dest_sz)
    cv_crop = np.array(crop, dtype=np.float32)

    # Normalize intensity
    average_intensity = int(np.mean(cv_crop))
    max_intensity = np.max(cv_crop)
    min_intensity = np.min(cv_crop)
    cv_crop = (cv_crop - average_intensity) * (target_range / max(1.0, max_intensity - min_intensity)) + target_intensity

    return cv_crop, left_eye, right_eye


# given rectangle (x, y, w, h) return the center point (cx, cy)
def get_center(rect):
    (x, y, w, h) = rect
    return (x + w/2, y + h/2)


if __name__ == '__main__':
    try:
	node = FaceDetectNode()

	atexit.register(node.on_exit)
	node.run()
    except rospy.ROSInterruptException:
	pass


