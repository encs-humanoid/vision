#!/usr/bin/python
from __future__ import division
import calc_pixelprint_precision
import face_util
import pub_face
import random
import rospy
import sys
import time

if len(sys.argv) >= 2:
    face_names_file = sys.argv[1]

    pub_face = pub_face.PubFaceNode("/detected_face", "/recognized_face", "/unrecognized_face")

    with open(face_names_file, 'r') as f:
	lines = f.readlines()

    face_names = [a.split() for a in lines]

    train = [(a[0].replace("facedb/train", "facedb"), a[1]) for a in face_names if "/train/" in a[0]]
    cv = [(a[0].replace("facedb/cv", "facedb.bak/cv"), a[1]) for a in face_names if "/cv/" in a[0]]

    pub_face.rate.sleep()
    time.sleep(3)  # allow time for subscribers to connect
    pub_face.rate.sleep()

    recog_sdr_names = []
    for image_file, name in train:
    	# publish the image to the recognizer
	pub_face.publish_face(image_file, random.randrange(1, 550), random.randrange(1, 350))

	# wait for a recognized or unrecognized response
	pub_face.wait_for_recognition()

	# if recognized, store the encounter ids and name
	if pub_face.received_recog_face:
	    recog_sdr_names.append((image_file, name, pub_face.recognized_face))
	# if unrecognized, print and continue -- shouldn't happen
	elif pub_face.received_unrec_face:
	    print image_file, name, "not recognized"

    print "ImageFile", "Name", "Threshold", "Precision", "Recall"

    # for each cv image
    for image_file, name in cv:
    	# publish the image to the recognizer
	pub_face.publish_face(image_file)

	# wait for a recognized or unrecognized response
	pub_face.wait_for_recognition()

	# if recognized, match the encounter ids to all recog_sdr_names => overlap
	if pub_face.received_recog_face:
	    # threshold overlap to get result
	    recognized_face = pub_face.recognized_face
	    bits = set(recognized_face.encounter_ids)
	    overlap = [(im, nm, len(bits & set(f.encounter_ids))) for im, nm, f in recog_sdr_names]
	    # match names of result to cv to get precision and recall
	    for threshold in [2, 4, 6, 8, 10, 12, 14, 16, 18, 20]:
		result = [(im, nm, sim) for im, nm, sim in overlap if sim >= threshold]
		precision, recall = calc_pixelprint_precision.calc_precision_recall(overlap, threshold, name)
		print image_file, name, str(threshold), str(precision), str(recall)
	# if unrecognized, set precision and recall to 0
	elif pub_face.received_unrec_face:
	    for threshold in [2, 4, 6, 8, 10, 12, 14, 16, 18, 20]:
		print image_file, name, str(threshold), "0", "0"
else:
    print "Usage: <face_names.txt>"


