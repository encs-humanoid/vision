#!/usr/bin/python
#===================================================================
# Wrapper to access face database.
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import division
from __future__ import print_function
import os
import face_util
import glob


class Face(object):
    def __init__(self, encounter_id, image_file, max_overlap, avg_overlap, min_overlap):
    	self.encounter_id = encounter_id
    	self.image_file = image_file
    	self.max_overlap = max_overlap
    	self.avg_overlap = avg_overlap
    	self.min_overlap = min_overlap
	self.bits = None


    def get_bits(self):
    	if self.bits is None:
	    self.bits = face_util.get_bits(self.image_file)
	return self.bits


class FaceEncounter(object):
    def __init__(self, encounter_folder, encounter_id):
    	self.encounter_folder = encounter_folder
	self.encounter_id = encounter_id


    def iterate(self):
    	'''
	Returns a generator for Faces within the face encounter.
	'''
	faces_txt = os.path.join(self.encounter_folder, "faces.txt")
	if os.path.isfile(faces_txt):
	    with open(faces_txt, "r") as f:
		lines = f.readlines()
	    for line in [l.strip('\n') for l in lines]:
		tokens = line.split()
		image_file = tokens[0]
		max_overlap = int(tokens[2])
		avg_overlap = int(tokens[3])
		min_overlap = int(tokens[4])
		yield Face(self.encounter_id, image_file, max_overlap, avg_overlap, min_overlap)
	else:
	    raise Exception("File not found: " + faces_txt)


class FaceDB(object):
    def __init__(self, facedb_path):
    	self.facedb_path = facedb_path

    def iterate(self):
    	'''
	Returns a generator for FaceEncounters within the face database.
	'''
    	for encounter_folder in glob.iglob(os.path.join(self.facedb_path, "*")):
	    encounter_id = encounter_folder.split("/")[-1]
	    if encounter_id.isdigit():  # skip any which are not digits
	    	encounter_id = int(encounter_id)
	    	encounter = FaceEncounter(encounter_folder, encounter_id)
		yield encounter

