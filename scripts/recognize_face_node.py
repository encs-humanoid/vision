#!/usr/bin/python
#===================================================================
# This is the Recognize Face Node.
#
# Subscribes To:
# - /detected_face
# - /control
#
# Publishes To:
# - /recognized_face
#
# Given a detected face, calculate the best match to the images of
# people encountered previously.
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import division
from vision.msg import DetectedFace
from vision.msg import LearnedFace
from vision.msg import RecognizedFace
from cv_bridge import CvBridge, CvBridgeError
from time import sleep
import argparse
import atexit
import cv2
import face_util
import Image
import glob
import heapq
import multiprocessing
import numpy as np
import os
import Queue
import random
import rospy
import sensor_msgs.msg
import std_msgs.msg
import sys


class RecognizeFaceNode(object):
    def __init__(self):
	rospy.init_node('recognize_face_node')

	self.bridge = CvBridge()

	self.last_detected_face = None
	self.is_recognizing = True

	detect_topic = self.get_param('~in_detect', '/detected_face')
	learn_topic = self.get_param('~in_learn', '/learned_face')
	control_topic = self.get_param('~in_control', '/control')
	rec_topic = self.get_param('~out_rec', '/recognized_face')
	unrec_topic = self.get_param('~out_unrec', '/unrecognized_face')
	self.min_match = int(self.get_param('~min_match', '4'))
	self.max_processes = int(self.get_param('~max_processes', '4'))
	self.threshold = int(self.get_param('~threshold', '270'))
	self.facedb_path = self.get_param('~facedb', 'facedb')

	self.start_processors()
	self.load_facedb()

	rospy.Subscriber(detect_topic, DetectedFace, self.on_detected_face)
	rospy.Subscriber(learn_topic, LearnedFace, self.on_learned_face)
	rospy.Subscriber(control_topic, std_msgs.msg.String, self.on_control)
	self.rec_pub = rospy.Publisher(rec_topic, RecognizedFace, queue_size=50)
	self.unrec_pub = rospy.Publisher(unrec_topic, DetectedFace, queue_size=50)

#	self.names = read_known_people(self.facedb_path, "names.txt")


    def get_param(self, param_name, param_default):
	value = rospy.get_param(param_name, param_default)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param_name), value)
	return value


    def start_processors(self):
	num_processors = min(multiprocessing.cpu_count() * 2, self.max_processes)
	self.processors = [Processor(self.threshold) for i in xrange(num_processors)]
	self.processes = []
	for w in self.processors:
	    self.processes.append(w)
	    w.daemon = False
	    w.start()


    def load_facedb(self):
	self.next_processor_index = 0
	rospy.loginfo("loading face database")
	rospy.loginfo('Current working directory %s', os.getcwd())
	rospy.loginfo('Searching facedb %s', os.path.join(self.facedb_path, "*"))
	for encounter_folder in glob.iglob(os.path.join(self.facedb_path, "*")):
	    encounter_id = encounter_folder.split("/")[-1]
	    if encounter_id.isdigit(): # skip any which are not digits
		faces_txt = os.path.join(encounter_folder, "faces.txt")
		if os.path.isfile(faces_txt):
		    with open(faces_txt, "r") as f:
			lines = f.readlines()
		    count = 0
		    for line in [l.strip('\n') for l in lines]:
			if self.next_processor_index >= len(self.processors):
			    self.next_processor_index = 0
			self.processors[self.next_processor_index].db_queue.put(line)
			self.next_processor_index += 1
			count += 1
		    rospy.loginfo("loaded %d images from face encounter %s", count, encounter_id)
		else:
		    rospy.loginfo("Missing file %s", faces_txt)
	rospy.loginfo("done loading face database")


    def on_detected_face(self, face):
	# don't try to recognize every face message that arrives, or we may not
	# be able to keep up.  Just store the last face received, and the run
	# loop will process them as fast as it can.
	self.last_detected_face = face


    def on_learned_face(self, learned_face):
	# update the facedb with the learned face
	self.next_processor_index = self.next_processor_index % len(self.processors)
	self.processors[self.next_processor_index].db_queue.put(learned_face)
	self.next_processor_index += 1
	rospy.loginfo("on learned face %s file %s", str(learned_face.encounter_id), learned_face.filepath)


    def on_control(self, control_msg):
	if control_msg.data == "stop_face_recognition":
	    self.is_recognizing = False
	    rospy.loginfo("stopped face recognition")
	elif control_msg.data == "resume_face_recognition":
	    self.is_recognizing = True
	    rospy.loginfo("resumed face recognition")


    def run(self):
	rate = rospy.Rate(30) # 30 Hz
	try:
	    while not rospy.is_shutdown():
		rate.sleep() # give ROS a chance to run
		if self.last_detected_face:
		    # grab the last face we have received and clear the
		    # handle so we don't pick up the same face next time
		    face, self.last_detected_face = self.last_detected_face, None

		    if self.is_recognizing:
			self.recognize_face(face)
	except KeyboardInterrupt:
	    pass

	self.on_exit()


    def on_exit(self):
	# send None message to background processes to have them terminate
	for w in self.processors:
	    w.input_queue.put(None)

	for w in self.processes:
	    w.terminate()
	    w.join()


    def recognize_face(self, face):
	cv_image = self.bridge.imgmsg_to_cv2(face.image)
	test_bits = set(face_util.get_pixelprint(cv_image))
	
	num_top_matches = 150
	heap = [] # will store heap of tuples (overlap, Match)
	for processor in self.processors:
	    processor.reset()
	    processor.input_queue.put(test_bits)
	running = True
	while running:
	    running = False
	    for processor in self.processors:
		if processor.has_more():
		    running = True
		    try:
			result = processor.output_queue.get_nowait()
			if not result:
			    processor.finish()
			else:
			    item = (result.overlap, result)
			    if len(heap) < num_top_matches or result.overlap > heap[0][0]:
				if len(heap) == num_top_matches:
				    heapq.heappushpop(heap, item)
				else:
				    heapq.heappush(heap, item)
		    except Queue.Empty, e:
			pass

	list = [item[1] for item in heap]  # note: list is not sorted by overlap

	# get unique face encounter ids from the top matches in the heap
	ids = set([m.encounter_id for m in list])

	rospy.loginfo('Matched encounter ids %s', str(sorted(ids)))

	# publish recognition result
	if len(ids) >= self.min_match:
	    recognized_face = RecognizedFace()
	    recognized_face.header = face.header
	    recognized_face.track_id = face.track_id
	    recognized_face.track_color = face.track_color
	    recognized_face.x = face.x
	    recognized_face.y = face.y
	    recognized_face.w = face.w
	    recognized_face.h = face.h
	    recognized_face.encounter_ids = sorted(ids)
	    self.rec_pub.publish(recognized_face)
	else:
	    self.unrec_pub.publish(face)
	    
	# determine which person is most frequently returned in the num_top_matches matches
#	top = dict()
#	person_name = "unknown"
#	for i in xrange(min(num_top_matches, len(list))):
#	    person_name = list[i].file.split('/')[1]
#	    #print person_name + ", " + str(list[i].overlap)
#	    if person_name in top:
#		top[person_name] += 1
#	    else:
#		top[person_name] = 1
#	top_count = 0
#	top_name = None
#	for name in top.keys():
#	    if top[name] > top_count:
#		top_count = top[name]
#		top_name = name

#	top_name = "unknown"
#	top_overlap = 0
#	lst = []
#	for name in self.names.keys():
#	    bits = self.names[name]
#	    overlap = len(set(bits) & ids)
#	    lst.append((name, overlap))
#	    if overlap > top_overlap:
#		top_overlap = overlap
#		top_name = name
				    
#	print "Recognized " + top_name + ": " + str(lst)


def read_known_people(path, filename):
    with open(os.path.join(path, filename)) as f:
	lines = f.readlines()

    names = dict()
    for line in [l.strip('\n') for l in lines]:
	name, pattern = line.split(':')
	bits = eval(pattern)
	names[name] = bits

    return names


class Match:
    def __init__(self, overlap, encounter_id, filepath):
	self.overlap = overlap
	self.encounter_id = encounter_id
	self.filepath = filepath


class DBImage:
    def __init__(self, filepath, encounter_id, max_overlap, avg_overlap, min_overlap, fp):
	self.filepath = filepath
	self.encounter_id = encounter_id
	self.max_overlap = max_overlap
	self.avg_overlap = avg_overlap
	self.min_overlap = min_overlap
	self.fp = set(fp)


class Processor(multiprocessing.Process):
    def __init__(self, threshold):
	multiprocessing.Process.__init__(self)
	self.db_queue = multiprocessing.Queue()
	self.input_queue = multiprocessing.Queue()
	self.output_queue = multiprocessing.Queue()
	self._has_more = False
	self.cwd = os.getcwd()
	self.image_database = []
	self.threshold = threshold


    def reset(self):
	self._has_more = True


    def has_more(self):
	return self._has_more


    def finish(self):
	self._has_more = False


    def run(self):
    	valid_encounter_threshold = 65
    	os.chdir(self.cwd)
	try:
	    while True:
		# check for new images to add to the database shard for this process
		try:
		    db_object = self.db_queue.get_nowait()
		    if isinstance(db_object, basestring):
			fields = db_object.split("\t")
			db_file = fields[0]
			encounter_id = int(fields[1])
			max_overlap = int(fields[2])
			avg_overlap = int(fields[3])
			min_overlap = int(fields[4])
			db_image = Image.open(db_file)
			cv_image = np.array(db_image, dtype=np.float32)
			db_bits = set(face_util.get_pixelprint(cv_image))
			# min_overlap < 65 usually implies a bad image in the database, which
			# can hamper recognition.  exclude any encounters with a low minimum
			if min_overlap >= valid_encounter_threshold:
			    self.image_database.append(DBImage(db_file, encounter_id, max_overlap, avg_overlap, min_overlap, db_bits))
			#print "added encounter " + str(encounter_id) + " file " + db_file
		    elif isinstance(db_object, LearnedFace):
			f = db_object
			# min_overlap < 65 usually implies a bad image in the database, which
			# can hamper recognition.  exclude any encounters with a low minimum
			if f.min_overlap >= valid_encounter_threshold:
			    self.image_database.append(DBImage(f.filepath, f.encounter_id, f.max_overlap, f.avg_overlap, f.min_overlap, f.bits))
			#print "added encounter " + str(f.encounter_id) + " file " + f.filepath
		    continue
		except Queue.Empty, e:
		    pass

		# wait for a test recognition to come, process it, and send back the match results
		try:
		    test_bits = self.input_queue.get_nowait()
		    if not test_bits: # use None as flag to terminate the process
			break
		    for item in self.image_database:
			overlap = len(item.fp & test_bits)
			# limit the results to matches which exceed a threshold relative to the face encounter
			#if overlap >= (item.avg_overlap + item.min_overlap) / 2:
			# Use absolute threshold to filter recognized vs. not
			if overlap >= self.threshold:
			    self.output_queue.put(Match(overlap, item.encounter_id, item.filepath))
		    self.output_queue.put(None)
		except Queue.Empty, e:
		    sleep(0.001)
	except KeyboardInterrupt:
	    pass


if __name__ == '__main__':
    try:
	node = RecognizeFaceNode()

	#atexit.register(node.on_exit)
	node.run()
    except rospy.ROSInterruptException:
	pass


