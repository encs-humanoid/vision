#!/usr/bin/python
#===================================================================
# This is the Learn Face Node.
#
# Subscribes To:
# - /detected_face
#
# Publishes To:
# - 
#
# Analyze images of human faces detected in a camera video stream to
# build a model that distinguishes the faces of individuals.
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
import os
import random
import rospy
import sensor_msgs.msg
import sys


### TODO 
### compute the average encounter overlap and store
### publish message to notify recognizer of a new encounter


# The upper limit for the encounter id.  These are assigned randomly,
# so two unrelated encounters may get the same id.  It is expected that
# an individual will have several encounters during face learning, and 
# it is the combination of several random encounter ids, not the individual
# id that is used to identify a person.
MAX_ENCOUNTER_ID = 2000


class FaceEncounter(object):
    def __init__(self, face=None):
	self.id = random.randrange(MAX_ENCOUNTER_ID)
	self.faces = []
	self.save_index = 0
	if face:
	    self.append(face)


    def __len__(self):
    	return len(self.faces)


    def is_related_to(self, face):
	'''
	return true if the given face is related to this encounter or this encounter
	is empty, false otherwise
	'''
        global delta_xy_px, delta_t_ms
        # an encounter is defined by a set of images which occur close in
        # time and space.
	if len(self.faces) > 0:
	    prev = self.faces[-1]
	    delta_x = abs(face.x - prev.x)
	    delta_y = abs(face.y - prev.y)
	    delta_t = abs((face.header.stamp - prev.header.stamp).to_sec() * 1000)
	    #print face.header.frame_id + " dx=" + str(delta_x) + ", dy=" + str(delta_y) + ", dt=" + str(delta_t)
	    if delta_x <= delta_xy_px and delta_y <= delta_xy_px and delta_t <= delta_t_ms:
	        return True
	    else:
	    	return False
	else:
	    return True


    def append(self, face):
	'''
	append the given face to this encounter
	'''
	self.faces.append(face)


    def save(self):
	'''
	save this encounter to disk
	The encounter is represented by a folder named after the encounter id.
	The faces are stored as images within the folder.
	'''
	global facedb_path, bridge
	encounter_path = os.path.join(facedb_path, str(self.id))
	if self.save_index == 0:
	    mkdir(encounter_path)
	for i, face in enumerate(self.faces[self.save_index:]):
	    try:
		frame_id = face.header.frame_id.replace("/", "_")
		name = str(face.header.stamp) + frame_id + "_" + str(self.save_index + i)
		image_name = os.path.join(encounter_path, name + ".png")
		cv_image = bridge.imgmsg_to_cv2(face.image)
		cv2.imwrite(image_name, cv_image)
		#print "wrote file " + image_name
	    except CvBridgeError, e:
		print e
	self.save_index = len(self.faces)


    def is_expired(self, face):
    	'''
	return true if the timestamp of the last face in the encounter is more
	that the time tolerance relative to the given face, else return false
	'''
        global delta_t_ms
	if self.faces:
	    prev = self.faces[-1]
	    return abs((face.header.stamp - prev.header.stamp).to_sec() * 1000) > delta_t_ms
	else:
	    return False


class LearnFaceNode(object):
    def __init__(self):
        global delta_xy_px, delta_t_ms, bridge, facedb_path

	rospy.init_node('learn_face_node')

	bridge = CvBridge()

	self.last_detected_face = None
	self.encounters = {} # dict keyed on camera frame_id containing list of FaceEncounter

	myargs = rospy.myargv(sys.argv) # process ROS args and return the rest
	parser = argparse.ArgumentParser(description="Learn faces in a ROS image stream")
	self.options = parser.parse_args(myargs[1:])

	input_topic = self.get_param('~in', '/detected_face')
	delta_xy_px = int(self.get_param('~delta_xy_px', '20'))
	delta_t_ms = int(self.get_param('~delta_t_ms', '1000'))
	facedb_path = self.get_param('~facedb', 'facedb')
	
	face_sub = rospy.Subscriber(input_topic, DetectedFace, self.on_detected_face)


    def get_param(self, param_name, param_default):
	value = rospy.get_param(param_name, param_default)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param_name), value)
	return value


    def on_detected_face(self, face):
	self.last_detected_face = face


    def run(self):
	rate = rospy.Rate(30) # 30 Hz
	try:
	    while not rospy.is_shutdown():
		rate.sleep() # give ROS a chance to run
		if self.last_detected_face:
		    # grab the last face we have received and clear the
		    # handle so we don't pick up the same face next time
		    face, self.last_detected_face = self.last_detected_face, None

		    self.process_face(face)
	except KeyboardInterrupt:
	    pass

	self.on_exit()


    def on_exit(self):
	pass


    def process_face(self, face):
	# keep images from different cameras separate
	if not face.header.frame_id in self.encounters:
	    self.encounters[face.header.frame_id] = []
	encounters_for_frame = self.encounters[face.header.frame_id]

	if len(encounters_for_frame) == 0:
	    # if list is empty store the face in a new encounter
	    encounters_for_frame.append(FaceEncounter(face))
	else:
	    to_remove = []
	    for i, encounter in enumerate(encounters_for_frame):
		# determine if the encounter should be retained in the list
		if encounter.is_expired(face):
		    to_remove.append(i)
	    	# determine if face belongs to the encounter
		elif encounter.is_related_to(face):
		    #print "adding face to encounter " + str(encounter.id)
		    encounter.append(face)
		    # determine if encounter has 5 faces and write it out
		    if len(encounter) >= 5:
		    	encounter.save()
		    face = None
		    break

	    if face:
		# face doesn't match existing encounters, so create new one
		encounters_for_frame.append(FaceEncounter(face))

	    # remove expired encounters
	    for i in reversed(to_remove):
		del encounters_for_frame[i]


def mkdir(path):
    if not os.path.isdir(path):
	os.makedirs(path)


if __name__ == '__main__':
    try:
	node = LearnFaceNode()

	atexit.register(node.on_exit)
	node.run()
    except rospy.ROSInterruptException:
	pass


