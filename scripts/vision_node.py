#!/usr/bin/python
#===================================================================
# This is the Vision Node.
#
# Subscribes To:
# - /stereo/left/image_raw	left eye image stream
# - /stereo/right/image_raw	right eye image stream
# - /joints			neck pose
# - /control			feature control channel
# - /target_face		provides indication of face to track
#
# Publishes To:
# - /face			all faces found from haar cascade
# - /chessboard			PointCloud of chessboard corners
# - /detected_face		faces also having two detectable eyes
# - /detected_face/image_raw	cropped and normalized image of face
# - /recognized_face		id pattern of each recognized person
# - /joy			joystick commands for face tracking
# - /eye_alignment		AlignEyes msg with offset and rotation
#
# On exit:
# - saves alignment to ./eye_alignment.p
#
# This node is part of a vision system capable of analyzing stereo
# image streams for human faces, learning to recognize individual
# people, publishing recognized person identifiers, and tracking
# a target person.
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import division
from cv_bridge import CvBridge, CvBridgeError
import atexit
import cv2
import face_util
import geometry_msgs.msg
import math
import numpy as np
import os
import pickle
import rospy
import sensor_msgs.msg
import std_msgs.msg
import sys
import vision.msg


# To-Do List
# annotate face node to show all detected face pairs simultaneously
# eliminate face_detect_node
# eliminate simple_face_tracker
# TODO track face by recognition degree
#   - TargetFace, if any
#   - else, RecognizedFace, if any
#   - else, DetectedFace, if any
#   - else, Face, if any
#   - else, look around
# TODO publish total number of detected faces after matching
# TODO move look around behavior from simple_face_tracker.py


DEFAULT_FACE_CASCADE = "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"
DEFAULT_EYE_CASCADE = "/usr/share/opencv/haarcascades/haarcascade_eye.xml"


# utility functions to calculate angle between vectors
# from: http://stackoverflow.com/a/13849249

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    angle = np.arccos(np.dot(v1_u, v2_u))
    if np.isnan(angle):
        if (v1_u == v2_u).all():
            return 0.0
        else:
            return np.pi
    return angle


class StereoFrame(object):

    """
    Object to hold matching image pairs from a stereo camera.
    """

    def __init__(self, left_camera_info, left_ros_image, right_camera_info, right_ros_image):
    	self.left_camera_info = left_camera_info
    	self.right_camera_info = right_camera_info
    	self.left_ros_image = left_ros_image
	self.right_ros_image = right_ros_image
	self._left_image = None
	self._right_image = None


    def has_camera_info(self):
    	return self.left_camera_info is not None and self.right_camera_info is not None


    def get_images(self):
    	""" return the frame's images in cv2 format. """
    	global bridge
	if self._left_image is None:
	    self._left_image = bridge.imgmsg_to_cv2(self.left_ros_image, "bgr8")
	if self._right_image is None:
	    self._right_image = bridge.imgmsg_to_cv2(self.right_ros_image, "bgr8")
    	return self._left_image, self._right_image


    def get_left_ts(self):
    	""" return the timestamp in seconds of the left image """
	return self.left_ros_image.header.stamp.to_sec()


    def get_right_ts(self):
    	""" return the timestamp in seconds of the right image """
	return self.right_ros_image.header.stamp.to_sec()


    def get_width(self):
    	""" return the width in pixels of the left camera, assuming the right camera is identical. """
    	return self.left_camera_info.width


    def get_height(self):
    	""" return the height in pixels of the left camera, assuming the right camera is identical. """
    	return self.left_camera_info.height


    def get_distortion_model(self):
    	""" return the distortion model of the left camera, assuming the right camera is identical. """
    	return self.left_camera_info.distortion_model


    def get_D(self):
    	""" return a tuple of (left D, right D) where D is the list of distortion parameters for the distortion model. """
	return (self.left_camera_info.D, self.right_camera_info.D)


    def get_K(self):
    	""" return a tuple of (left K, right K) where K is the intrinsic camera matrix (see sensor_msgs/CameraInfo). """
	return (np.reshape(self.left_camera_info.K, (3, 3)), np.reshape(self.right_camera_info.K, (3, 3)))


    def get_R(self):
    	""" return a tuple of (left R, right R) where R is the rectification matrix (see sensor_msgs/CameraInfo). """
	return (np.reshape(self.left_camera_info.R, (3, 3)), np.reshape(self.right_camera_info.R, (3, 3)))


    def get_P(self):
    	""" return a tuple of (left P, right P) where P is the camera projection matrix (see sensor_msgs/CameraInfo). """
	return (np.reshape(self.left_camera_info.P, (3, 4)), np.reshape(self.right_camera_info.P, (3, 4)))
    	

class StereoAlignment(object):
    """
    This class holds the alignment parameters and supports mapping image coordinates
    from left to right and vice versa.
    """

    def __init__(self, height, translation, angle, left_center, right_center):
	self.h = height # image frame height
    	self.t = translation  # list-like with 2 elements (dx, dy) in image coordinates
	self.a = angle  # the relative rotation angle between the cameras in radians
	self.lc = left_center  # left_center + translation = right_center
	self.rc = right_center  # right_center - translation = left_center


    def map_image_points(self, points, is_right_to_left=True):
    	"""
	Map points in image coordinates from right camera to left camera image coordinates
	(or left to right if is_right_to_left is False) and return the list of coordinate pairs.
	"""
	# invert y-axis coordinates to convert from image to cartesian coordinates
	coords = np.array([[p[0], self.h - p[1]] for p in points])

	# alias the alignment parameters to more convenient names
	tx = self.t[0]
	ty = self.t[1]
	th = self.a
	lcx = self.lc[0]
	lcy = self.lc[1]
	rcx = self.rc[0]
	rcy = self.rc[1]

	# set up the transformation for right to left or left to right
	if is_right_to_left:
	    t = np.array([-tx, ty])
	    angle = -th
	    center = [lcx, self.h - lcy]
	else:
	    t = np.array([tx, -ty])
	    angle = th
	    center = [rcx, self.h - rcy]

	# compute the rotation matrix
	c = np.cos(angle)
	s = np.sin(angle)
	rot = np.array([[c, -s], [s, c]])

	# rotate and translate the coordinates
	mapped_coords = [np.dot(rot, p - center) + center + t for p in coords]

	# invert the y-axis to return to image coordinates
	mapped_points = [[x, self.h - y] for x, y in mapped_coords]

	# restore any extra columns (e.g., width and height) from the original points list
	mapped_points_with_extra = [[p1[0], p1[1]] + p2[2:].tolist() for p1, p2 in zip(mapped_points, points)]
	return mapped_points_with_extra


class StereoVisionOptions(object):
    """
    Container class for configuration options to the StereoVision class
    """

    def __init__(self, 
    		face_cascade_path=DEFAULT_FACE_CASCADE, 
		scale_factor=1.2, 
		min_neighbors=5, 
		min_size=(30, 30), 
		eye_cascade_path=DEFAULT_EYE_CASCADE,
		eye_scale_factor=1.1,
		eye_min_neighbors=5,
		eye_min_size=(5, 5),
		moving_avg_frames=20,
		chessboard_size=(7, 6),
		delta_xy_px=20,
		delta_t_ms=1000):
	self.face_cascade = cv2.CascadeClassifier(face_cascade_path)
	self.scale_factor = scale_factor
	self.min_neighbors = min_neighbors
	self.min_size = min_size
	self.eye_cascade = cv2.CascadeClassifier(eye_cascade_path)
	self.eye_scale_factor = eye_scale_factor
	self.eye_min_neighbors = eye_min_neighbors
	self.eye_min_size = eye_min_size
	self.moving_avg_frames = moving_avg_frames
	self.chessboard_size = chessboard_size
	self.delta_xy_px = delta_xy_px
	self.delta_t_ms = delta_t_ms


class NormalizedFace(object):
    def __init__(self, x, y, w, h, ts, left_eye, right_eye, image):
    	self.x = x
	self.y = y
	self.w = w
	self.h = h
	self.ts = ts
	self.left_eye = left_eye
	self.right_eye = right_eye
	self.image = image
	self.track = 0
	self.track_color = (255, 0, 0)


    def set_track(self, track):
    	self.track = track


class TrackedFace(object):

    next_id = 1  # class variable for next tracked face id

    def __init__(self, face, id=0):
    	if id == 0:
	    self.id = TrackedFace.next_id
	    TrackedFace.next_id += 1
	else:
	    self.id = id
	self.faces = [face]
	self.color = tuple(np.random.randint(0, 255, 3).tolist())


    def is_related_to(self, face, delta_xy_px, delta_t_ms):
	prev = self.faces[-1]  # compare proximity with most recent tracked face
	delta_x = abs(face.x - prev.x)
	delta_y = abs(face.y - prev.y)
	delta_w = abs(face.w - prev.w)
	delta_h = abs(face.h - prev.h)
	delta_t = abs((face.ts - prev.ts) * 1000)
	if delta_x <= delta_xy_px and \
	   delta_y <= delta_xy_px and \
	   delta_w <= delta_xy_px and \
	   delta_h <= delta_xy_px and \
	   delta_t <= delta_t_ms:
	    return True
	else:
	    return False


class StereoVision(object):
    """
    The StereoVision class processes double frames from a stereo camera pair.
    
    The first feature is to find faces in both eye frames and match the face
    pairs that appear in both eyes, keeping separate any faces that appear in
    only one eye or the other.
    """

    def __init__(self, options=StereoVisionOptions()):
    	self.options = options
	self.left_faces = []
	self.right_faces = []
	self.matched_faces = []
	self.found_left_chessboard = False
	self.found_right_chessboard = False
	self.left_chessboard_corners = []
	self.right_chessboard_corners = []
	self.left_right_translation_avg = None
	self.left_right_rotation_avg = None
	self.left_center_avg = None
	self.right_center_avg = None
	self.eye_alignment = None
	self.left_tracked_faces = []
	self.right_tracked_faces = []


    def process_frame(self, stereo_frame):
	# look for chessboard and update alignment, if found
	self.update_stereo_alignment(stereo_frame)

	# convert to grayscale
    	left_image, right_image = stereo_frame.get_images()
	left_gray_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
	right_gray_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

	# find faces
	self.left_faces = self.find_faces(left_gray_image)
	self.right_faces = self.find_faces(right_gray_image)
	#rospy.loginfo('got ' + str(len(self.left_faces)) + ' left and ' + str(len(self.right_faces)) + ' right faces')

	# for each stereo face pair, look for two eyes
	# if two eyes found, crop and normalize both images in the pair
	left_ts = stereo_frame.get_left_ts()
	right_ts = stereo_frame.get_right_ts()
	l, m, r = self.find_detected_faces(self.left_faces, left_gray_image, self.right_faces, right_gray_image, left_ts, right_ts)
	self.left_detected_faces = l
	self.matched_faces = m
	self.right_detected_faces = r

	# for each face or face pair in the current stereo frame
	# track the face by proximity to faces in previous frames
	# a TrackedFace is first created when a face appears in view where no other face has been for at least delta_t_ms
	# Each time a face is detected within proximity of a TrackedFace, it is added to the TrackedFace
	# Every DetectedFace becomes a TrackedFace either by starting a new one or joining an existing one
	# The DetectedFace track is assigned from the TracedFace id.
	self.track_faces(l, m, r)


    def update_stereo_alignment(self, stereo_frame):
    	"""
	Given a stereo frame, if a chessboard pattern can be found in both images, compute
	the translation and rotation between the image chessboards as an approximation of
	the transformation between the stereo frames.  Update a moving average of such
	transformations, which is used for mapping between the frames.
	"""
    	left_image, right_image = stereo_frame.get_images()
	self.found_left_chessboard, self.left_chessboard_corners = cv2.findChessboardCorners(left_image, self.options.chessboard_size, flags=cv2.CALIB_CB_FAST_CHECK)
	#rospy.loginfo('left corners ' + str(self.left_chessboard_corners))
	self.found_right_chessboard, self.right_chessboard_corners = cv2.findChessboardCorners(right_image, self.options.chessboard_size, flags=cv2.CALIB_CB_FAST_CHECK)
	#rospy.loginfo('right corners ' + str(self.right_chessboard_corners))

	if self.found_left_chessboard and self.found_right_chessboard:
	    # calculate the translation to align the chessboard centers
	    left_center = np.average(self.left_chessboard_corners, axis=0)[0]
	    #rospy.loginfo('left center ' + str(left_center))
	    right_center = np.average(self.right_chessboard_corners, axis=0)[0]
	    #rospy.loginfo('right center ' + str(right_center))

	    if self.left_center_avg is None or self.right_center_avg is None:
	    	self.left_center_avg = left_center
		self.right_center_avg = right_center
	    else:
	    	m = self.options.moving_avg_frames
	    	self.left_center_avg = (m - 1.0) / m * self.left_center_avg + 1.0 / m * left_center
		self.right_center_avg = (m - 1.0) / m * self.right_center_avg + 1.0 / m * right_center

	    left_to_right = right_center - left_center
	    lv0 = (self.left_chessboard_corners[0][0] - left_center)
	    #rospy.loginfo('lv0 ' + str(lv0))
	    rv0 = (self.right_chessboard_corners[0][0] - right_center)
	    #rospy.loginfo('rv0 ' + str(rv0))
	    angle0 = angle_between(lv0, rv0)
	    #rospy.loginfo('translation vector ' + str(left_to_right) + ", rotation angle " + str(angle0*180/math.pi))

	    if self.left_right_translation_avg is None or self.left_right_rotation_avg is None:
	    	self.left_right_translation_avg = left_to_right
		self.left_right_rotation_avg = angle0
	    else:
	    	m = self.options.moving_avg_frames
	    	self.left_right_translation_avg = (m - 1.0) / m * self.left_right_translation_avg + 1.0 / m * left_to_right
		self.left_right_rotation_avg = (m - 1.0) / m * self.left_right_rotation_avg + 1.0 / m * angle0
	    self.eye_alignment = StereoAlignment(stereo_frame.get_height(), self.left_right_translation_avg, self.left_right_rotation_avg, self.left_center_avg, self.right_center_avg)
	    rospy.loginfo('avg translation vector ' + str(self.left_right_translation_avg) + ", avg rotation angle " + str(self.left_right_rotation_avg*180/math.pi))


    def find_faces(self, gray_image):
    	""" Apply the Haar cascade to find faces in the grayscale image. """
	faces = self.options.face_cascade.detectMultiScale(
	    gray_image,
	    scaleFactor=self.options.scale_factor,
	    minNeighbors=self.options.min_neighbors,
	    minSize=self.options.min_size,
	    flags=cv2.cv.CV_HAAR_SCALE_IMAGE
	)
	return faces


    def find_detected_faces(self, left_faces, left_gray_image, right_faces, right_gray_image, left_ts, right_ts):
    	"""
	Filter the faces found for those with exactly two detectable eyes.
	Map the face coordinates from the right image to the left image coordinate system
	using the eye alignment transform and identify matches where the same face is
	detected in both image frames.  Return the left, matched, and right faces as
	disjoint sets.
	"""
	left_detected_faces = self.find_normalized_faces(left_faces, left_gray_image, left_ts)
	right_detected_faces = self.find_normalized_faces(right_faces, right_gray_image, right_ts)
	matched_faces = []

	# if calibrated, map right faces to left coordinate frame
	if self.eye_alignment is not None:
	    r_points = np.array([(f.x, f.y, f.w, f.h) for f in right_detected_faces])
	    mapped_points = self.eye_alignment.map_image_points(r_points)

	    # find stereo face pairs
	    if len(left_detected_faces) > 0 and len(right_detected_faces) > 0:
		matcher = cv2.BFMatcher(cv2.NORM_L2SQR, crossCheck=True)
		l_points = np.array([(f.x, f.y, f.w, f.h) for f in left_detected_faces])
		matches = matcher.match(np.array(l_points, dtype=np.float32), np.array(mapped_points, dtype=np.float32))
		#if len(matches) > 0:
		#    print [(m.queryIdx, m.trainIdx, m.distance) for m in matches]
		# organize faces into left-only, matched, right-only
		remove_from_left = []
		remove_from_right = []
		for m in matches:
		    remove_from_left.append(m.queryIdx)
		    remove_from_right.append(m.trainIdx)
		    matched_faces.append((left_detected_faces[m.queryIdx], right_detected_faces[m.trainIdx]))
		remove_from_left = sorted(remove_from_left, reverse=True)
		remove_from_right = sorted(remove_from_right, reverse=True)
		for i in remove_from_left:
		    del left_detected_faces[i]
		for i in remove_from_right:
		    del right_detected_faces[i]

	return left_detected_faces, matched_faces, right_detected_faces


    def find_normalized_faces(self, faces, gray_image, ts):
    	"""
	Search for eyes within a face image.  If found, use the eye positions
	to crop and normalize the face image.
	"""
    	normalized_faces = []
	for (x, y, w, h) in faces:
	    # Extract just the face as a subimage
	    face_only_gray = gray_image[y:y + h, x:x + w]

	    # look for eyes only in the top of the image
	    eye_band_upper=0.2
	    eye_band_lower=0.6
	    eyes = self.find_eyes(face_only_gray[eye_band_upper*h:eye_band_lower*h, :])

	    # since eyes are found in a band, shift the coordinates to be relative to the face
	    eyes = [(ex, ey + int(eye_band_upper*h), ew, eh) for (ex, ey, ew, eh) in eyes]

	    if len(eyes) == 2:  # keep only faces with exactly 2 detectable eyes
		cv_crop, left_eye, right_eye = face_util.crop_and_normalize(face_only_gray, eyes)
		normalized_faces.append(NormalizedFace(x, y, w, h, ts, left_eye, right_eye, cv_crop))
		
	return normalized_faces


    def find_eyes(self, face_only_gray):
    	""" Apply the Haar cascade to locate eyes within the image. """
	eyes = self.options.eye_cascade.detectMultiScale(face_only_gray,
		    scaleFactor=self.options.eye_scale_factor,
		    minNeighbors=self.options.eye_min_neighbors,
		    minSize=self.options.eye_min_size,
		    flags=cv2.cv.CV_HAAR_SCALE_IMAGE
	)
	return eyes


    def track_faces(self, left, matched, right):
    	"""
	Set the tracking id for each face.
	1. A newly appeared face gets a new tracking id assigned.
	2. A face in close proximity to a previously seen face is assigned the same tracking id as the previous face.
	3. Both faces in a face pair are assigned the same tracking id.
	"""
	self.update_face_tracking(left, self.left_tracked_faces)
	self.update_face_tracking(right, self.right_tracked_faces)
	self.update_face_tracking_for_pairs(matched)


    def update_face_tracking(self, faces, tracked_faces):
	tracks_to_add = []
	tracks_to_remove = []
	delta_xy_px = self.options.delta_xy_px
	delta_t_ms = self.options.delta_t_ms
	for face in faces:
	    # compare the face to the tracked faces
	    for track in tracked_faces:
		if len(track.faces) > 0:
		    if track.is_related_to(face, delta_xy_px, delta_t_ms):
			face.track = track.id
			face.track_color = track.color
			track.faces.append(face)
			break
		    elif face.ts - track.faces[-1].ts > 2:  # expire track after 2 seconds
		    	tracks_to_remove.append(track)  # could add same track more than once

	    if face.track == 0:  # no matching track found
	    	# create a new track and assign its id to the face
	    	track = TrackedFace(face)
		face.track = track.id
		face.track_color = track.color
		# store the track for comparison with future faces
	    	tracks_to_add.append(track)
		rospy.loginfo('Added new face track %d', face.track)
	    else:
		rospy.loginfo('Matched face to track %d', face.track)

	# remove expired tracks
	for track in tracks_to_remove:
	    if track in tracked_faces:  # may have already been removed
		tracked_faces.remove(track)

	# add new tracks
	for track in tracks_to_add:
	    tracked_faces.append(track)


    def update_face_tracking_for_pairs(self, matched):
    	left_tracks_to_add = []
	right_tracks_to_add = []
	delta_xy_px = self.options.delta_xy_px
	delta_t_ms = self.options.delta_t_ms
	for pair in matched:
	    left_face = pair[0]
	    for track in self.left_tracked_faces:
	    	if len(track.faces) > 0:
		    if track.is_related_to(left_face, delta_xy_px, delta_t_ms):
			rospy.loginfo('Matched face pair to left track %d', track.id)
			pair[0].track = track.id
			pair[1].track = track.id
			pair[0].track_color = track.color
			pair[1].track_color = track.color
			break

	    right_face = pair[1]
	    if right_face.track == 0:
		for track in self.right_tracked_faces:
		    if len(track.faces) > 0:
			if track.is_related_to(right_face, delta_xy_px, delta_t_ms):
			    rospy.loginfo('Matched face pair to right track %d', track.id)
			    pair[0].track = track.id
			    pair[1].track = track.id
			    pair[0].track_color = track.color
			    pair[1].track_color = track.color
			    break

	    if left_face.track == 0 or right_face.track == 0:  # no matching track found
	    	# create a new track and assign its id to the face
	    	left_track = TrackedFace(left_face)
		right_track = TrackedFace(right_face, left_track.id)
		right_track.color = left_track.color
		left_face.track = left_track.id
		right_face.track = right_track.id  # must be same as left_track
		left_face.track_color = left_track.color
		right_face.track_color = right_track.color  # same as left_track
		# store the track for comparison with future faces
	    	left_tracks_to_add.append(left_track)
	    	right_tracks_to_add.append(right_track)
		rospy.loginfo('Added new face pair track %d', left_face.track)

	# add new tracks
	for track in left_tracks_to_add:
	    self.left_tracked_faces.append(track)
	for track in right_tracks_to_add:
	    self.right_tracked_faces.append(track)


class VisionNode(object):

    def __init__(self):
    	rospy.init_node('vision_node')

	self.left_camera_info = None
	self.right_camera_info = None
	self.left_images = []
	self.right_images = []

	myargs = rospy.myargv(sys.argv) # process ROS args and return the rest

	# subscriber topic parameters
	left_image_topic = self.get_param('~left', '/stereo/left/image_raw')
	right_image_topic = self.get_param('~right', '/stereo/right/image_raw')
	joints_topic = self.get_param('~joints', '/joints')
	control_topic = self.get_param('~control', '/control')
	target_topic = self.get_param('~target', '/target_face')

	# publisher topic parameters
	face_topic = self.get_param('~face', '/face')
	chessboard_topic = self.get_param('~chessboard', '/chessboard')
	align_topic = self.get_param('~align', '/eye_alignment')
	left_camera_info_topic = self.get_param('~left_camera_info', '/stereo/left/camera_info')
	right_camera_info_topic = self.get_param('~right_camera_info', '/stereo/right/camera/camera_info')
	# TODO remove unnecessary relay, since this node publishes left and right faces to the /detected_face topic
	detected_face_topic = self.get_param('~detected_face', '/detected_face')
	left_face_img_topic = self.get_param('~left_face_img', '/stereo/left/detected_face/image_raw')
	right_face_img_topic = self.get_param('~right_face_img', '/stereo/right/detected_face/image_raw')

	# other parameters
	self.eye_alignment_file = self.get_param('~eye_alignment', 'eye_alignment.p')
	self.fps = int(self.get_param('~fps', '15'))

	left_image_sub = rospy.Subscriber(left_image_topic, sensor_msgs.msg.Image, self.on_left_image)
	right_image_sub = rospy.Subscriber(right_image_topic, sensor_msgs.msg.Image, self.on_right_image)
	joints_sub = rospy.Subscriber(joints_topic, sensor_msgs.msg.JointState, self.on_joints)
	control_sub = rospy.Subscriber(control_topic, std_msgs.msg.String, self.on_control)
	target_sub = rospy.Subscriber(target_topic, vision.msg.TargetFace, self.on_target_face)
	linfo_sub = rospy.Subscriber(left_camera_info_topic, sensor_msgs.msg.CameraInfo, self.on_left_camera_info)
	rinfo_sub = rospy.Subscriber(right_camera_info_topic, sensor_msgs.msg.CameraInfo, self.on_right_camera_info)

	self.face_pub = rospy.Publisher(face_topic, vision.msg.Face, queue_size=50)
	self.chessboard_pub = rospy.Publisher(chessboard_topic, sensor_msgs.msg.PointCloud, queue_size=5)
	self.eye_align_pub = rospy.Publisher(align_topic, vision.msg.AlignEyes, queue_size=1)
	self.detected_face_pub = rospy.Publisher(detected_face_topic, vision.msg.DetectedFace, queue_size=50)
	self.left_face_img_pub = rospy.Publisher(left_face_img_topic, sensor_msgs.msg.Image, queue_size=50)
	self.right_face_img_pub = rospy.Publisher(right_face_img_topic, sensor_msgs.msg.Image, queue_size=50)

	self.vision = StereoVision()

	# restore the previously saved eye alignment, if any
	self.load_eye_alignment()


    def get_param(self, param_name, param_default):
        value = rospy.get_param(param_name, param_default)
        rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param_name), value)
        return value


    def on_left_image(self, ros_image):
    	self.left_images.append(ros_image)


    def on_right_image(self, ros_image):
    	self.right_images.append(ros_image)


    def on_control(self, control):
    	# TODO implement staring control
        rospy.loginfo('Received control message %s', control)
	#if control.data == "resume_face_tracking":
	#    self.is_tracking = True
	#elif control.data == "stop_face_tracking":
	#    self.is_tracking = False


    def on_joints(self, joint_state):
    	# TODO implement look around
	pose = {}
	for i in range(len(joint_state.name)):
	    pose[joint_state.name[i]] = joint_state.position[i]
	# assign to self.pose in a single statement to avoid multithreading issue
	# (KeyError) with an incomplete dict
	#self.pose = pose
    	#self.last_joint_state = joint_state
        #rospy.loginfo('Received joints message')


    def on_target_face(self, target_face):
    	# TODO implement target tracking
	#self.last_target_face = target_face
	#self.last_target_face_ts = time.time()
        rospy.loginfo('Received target face message')


    def on_left_camera_info(self, camera_info):
    	self.left_camera_info = camera_info
    	#rospy.loginfo('Received left camera info ' + str(camera_info))


    def on_right_camera_info(self, camera_info):
    	self.right_camera_info = camera_info
    	#rospy.loginfo('Received right camera info ' + str(camera_info))


    def run(self):
	rate = rospy.Rate(30) # 30 Hz

	seconds_per_half_frame = 0.5 / self.fps

	try:
	    while not rospy.is_shutdown():
		rate.sleep() # give ROS a chance to run
		order = lambda msg: msg.header.stamp.to_sec()
		left_images = sorted(self.left_images, key=order)
		right_images = sorted(self.right_images, key=order)
		if len(left_images) > 0 and len(right_images) > 0:
		    # find most recent matching image frames from left and right cameras
		    i = len(left_images) - 1
		    j = len(right_images) - 1
		    while i >= 0 and j >= 0:
		    	left_sec = left_images[i].header.stamp.to_sec()
			right_sec = right_images[j].header.stamp.to_sec()
			if abs(left_sec - right_sec) < seconds_per_half_frame:
			    break  # found a pair
		    	if left_sec >= right_sec:
			    i -= 1
			else:
			    j -= 1
		    if i >= 0 and j >= 0:
			left_image = left_images[i]
			right_image = right_images[j]
			# clear image buffers, but keep any newer images for next round
			self.left_images[:] = left_images[i + 1:]
			self.right_images[:] = right_images[j + 1:]
			# process the stereo image frame
			stereo_frame = StereoFrame(self.left_camera_info, left_image, self.right_camera_info, right_image)
			self.vision.process_frame(stereo_frame)
			# publish faces
			self.publish_faces(stereo_frame)
			# publish detected faces and images
			self.publish_detected_faces(stereo_frame)
			# publish chessboards
			self.publish_chessboards(stereo_frame)
			self.publish_eye_alignment()
		else:
		    # TODO handle case of one eye closed
		    pass

	except KeyboardInterrupt:
	    pass


    def publish_faces(self, stereo_frame):
	# publish faces in left frame
	for f in self.vision.left_faces:
	    self.publish_face(stereo_frame.left_ros_image, f)

	# publish faces in right frame
	for f in self.vision.right_faces:
	    self.publish_face(stereo_frame.right_ros_image, f)


    def publish_face(self, image, face):
    	"""
	Publish a vision.msg.Face message for this face.
	The header for the Face message is copied from the image header.
	"""
    	# create Face message
	face_msg = vision.msg.Face()
	face_msg.header = image.header
	face_msg.x = face[0]
	face_msg.y = face[1]
	face_msg.w = face[2]
	face_msg.h = face[3]
	self.face_pub.publish(face_msg)


    def publish_detected_faces(self, stereo_frame):
	"""
	Publish a vision.msg.DetectedFace message for each detected face (face w/
	2 eyes, cropped, and normalized).
	"""
	for f in self.vision.left_detected_faces:
	    self.publish_detected_face(stereo_frame.left_ros_image, f, self.left_face_img_pub)

	# publish both matched faces in left frame
	for m in self.vision.matched_faces:
	    # copy coordinates from left face to right face
	    l = m[0]
	    r = NormalizedFace(l.x, l.y, l.w, l.h, l.ts, m[1].left_eye, m[1].right_eye, m[1].image)
	    r.track = l.track
	    self.publish_detected_face(stereo_frame.left_ros_image, r, self.left_face_img_pub)
	    self.publish_detected_face(stereo_frame.left_ros_image, l, self.left_face_img_pub)  # publish left face last on left channel

	# publish both matched faces in right frame
	for m in self.vision.matched_faces:
	    # copy coordinates from right face to left face
	    r = m[1]
	    l = NormalizedFace(r.x, r.y, r.w, r.h, r.ts, m[0].left_eye, m[0].right_eye, m[0].image)
	    l.track = r.track
	    self.publish_detected_face(stereo_frame.right_ros_image, l, self.right_face_img_pub)
	    self.publish_detected_face(stereo_frame.right_ros_image, r, self.right_face_img_pub)  # publish right face last on right channel

	for f in self.vision.right_detected_faces:
	    self.publish_detected_face(stereo_frame.right_ros_image, f, self.right_face_img_pub)


    def publish_detected_face(self, ros_image, face, detected_face_img_pub):
    	global bridge
	x = face.x
	y = face.y
	w = face.w
	h = face.h
	detected_face = face_util.create_detected_face_msg(bridge, x, y, w, h, face.left_eye, face.right_eye, face.image, ros_image)
	detected_face.track = face.track
	detected_face.track_color = face.track_color

	self.detected_face_pub.publish(detected_face)

	face_image = detected_face.image
	face_image.header.stamp = rospy.Time.now()

	detected_face_img_pub.publish(face_image)


    def publish_chessboards(self, stereo_frame):
    	if self.vision.found_left_chessboard:
	    self.publish_chessboard(stereo_frame.left_ros_image, self.vision.left_chessboard_corners)
	if self.vision.found_right_chessboard:
	    self.publish_chessboard(stereo_frame.right_ros_image, self.vision.right_chessboard_corners)


    def publish_chessboard(self, image, corners):
    	if len(corners) > 0:
	    cloud = sensor_msgs.msg.PointCloud()
	    cloud.header = image.header
	    cloud.points = [geometry_msgs.msg.Point32(p[0][0], p[0][1], 0) for p in corners]
	    cloud.channels = []
	    self.chessboard_pub.publish(cloud)


    def publish_eye_alignment(self):
    	if self.vision.left_right_translation_avg is not None and self.vision.left_right_rotation_avg is not None:
	    t = self.vision.left_right_translation_avg
	    align = vision.msg.AlignEyes()
	    align.p = geometry_msgs.msg.Point32(t[0], t[1], self.vision.left_right_rotation_avg)
	    align.lc = geometry_msgs.msg.Point32(self.vision.left_center_avg[0], self.vision.left_center_avg[1], 0.0)
	    align.rc = geometry_msgs.msg.Point32(self.vision.right_center_avg[0], self.vision.right_center_avg[1], 0.0)
	    self.eye_align_pub.publish(align)


    def load_eye_alignment(self):
        if os.path.exists(self.eye_alignment_file):
	    rospy.loginfo("Reading eye alignment from %s", self.eye_alignment_file)
	    with open(self.eye_alignment_file, "rb") as f:
		self.vision.eye_alignment = pickle.load(f)
	    self.vision.left_right_translation_avg = self.vision.eye_alignment.t
	    self.vision.left_right_rotation_avg = self.vision.eye_alignment.a
	    self.vision.left_center_avg = self.vision.eye_alignment.lc
	    self.vision.right_center_avg = self.vision.eye_alignment.rc


    def on_exit(self):
        rospy.loginfo("I received a kill signal")
    	if self.vision.eye_alignment:
	    rospy.loginfo("Writing eye alignment to %s", self.eye_alignment_file)
	    with open(self.eye_alignment_file, "wb") as f:
		pickle.dump(self.vision.eye_alignment, f)


if __name__ == '__main__':
    try:
	bridge = CvBridge()
	node = VisionNode()

	atexit.register(node.on_exit)
	node.run()
    except rospy.ROSInterruptException:
	pass


