#!/usr/bin/python
#===================================================================
# Wrapper to access face database.
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import division
from __future__ import print_function
import collections
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
    def __init__(self, encounter_folder, encounter_id, faces=None):
    	self.encounter_folder = encounter_folder
	self.encounter_id = encounter_id
	self.faces = faces
	self.subencounters = None


    def iterate(self):
    	'''
	Returns a generator for Faces within the face encounter.
	'''
	if self.faces:
	    for face in self.faces:
	    	yield face
	else:
	    faces_txt = os.path.join(self.encounter_folder, "faces.txt")
	    if os.path.isfile(faces_txt):
		with open(faces_txt, "r") as f:
		    lines = f.readlines()
		for line in [l.strip('\n') for l in lines]:
		    tokens = line.split()
		    image_file = os.path.join(self.encounter_folder, tokens[0].split('/')[-1])
		    max_overlap = int(tokens[2])
		    avg_overlap = int(tokens[3])
		    min_overlap = int(tokens[4])
		    yield Face(self.encounter_id, image_file, max_overlap, avg_overlap, min_overlap)
	    else:
		#print "File not found: " + faces_txt
		self.faces = []
		for png_file in glob.iglob(os.path.join(self.encounter_folder, "*.png")):
		    self.faces.append(Face(self.encounter_id, png_file, 0, 0, 0))

		self.calculate_ensemble_overlap()
		with open(faces_txt, "w") as f:
		    for face in self.faces:
			f.write("\t".join([face.image_file, str(face.encounter_id), str(face.max_overlap), str(face.avg_overlap), str(face.min_overlap)]) + "\n")

		for face in self.faces:
		    yield face


    def iterate_subencounters(self):
    	if self.subencounters is None:
	    # group faces within the encounter folder into lists by their overlap statistics
	    d = collections.defaultdict(list)
	    for face in self.iterate():
		key = "%s/%s/%s" % (face.max_overlap, face.avg_overlap, face.min_overlap)
		d[key].append(face)
	    
	    # yield an encounter for each group of faces
	    self.subencounters = []
	    for group in d.values():
		subencounter = FaceEncounter(self.encounter_folder, self.encounter_id, group)
		self.subencounters.append(subencounter)

	for subencounter in self.subencounters:
	    yield subencounter


    def calculate_ensemble_overlap(self):
    	if self.faces is None:
	    raise Exception("Overlap can only be calculated for a subencounter")
	faces = self.faces
	cnt_overlap = len(faces) * (len(faces) - 1) / 2
	max_overlap = 0
	avg_overlap = 0
	min_overlap = None
	for i in xrange(len(faces) - 1):
	    for j in xrange(i +  1, len(faces)):
		o = len(faces[i].get_bits() & faces[j].get_bits())
		max_overlap = max(max_overlap, o)
		avg_overlap += o / cnt_overlap
		if min_overlap:
		    min_overlap = min(min_overlap, o)
		else:
		    min_overlap = o
	# copy the overlap statistics to each face in the encounter
	for face in faces:
	    face.max_overlap = max_overlap
	    face.avg_overlap = int(avg_overlap)
	    face.min_overlap = min_overlap


class FaceDB(object):
    def __init__(self, facedb_path):
    	self.facedb_path = facedb_path

    def iterate(self):
    	'''
	Returns a generator for FaceEncounters within the face database.
	'''
	for encounter_folder, encounter_id in self.iterate_encounter_folder_ids():
	    encounter = FaceEncounter(encounter_folder, encounter_id)
	    yield encounter


    def iterate_encounter_folder_ids(self):
    	for encounter_folder in self._get_encounter_folders():
	    encounter_id = encounter_folder.split("/")[-1]
	    if encounter_id.isdigit():  # skip any which are not digits
	    	encounter_id = int(encounter_id)
		yield encounter_folder, encounter_id
    	

    def _get_encounter_folders(self):
    	return glob.iglob(os.path.join(self.facedb_path, "*"))


    def iterate_unique(self):
    	'''
	Returns a generator for FaceEncounters within the face database where encounters
	having the same id are broken into separate objects by the overlap statistics.
	'''
    	for encounter in self.iterate():
	    for subencounter in encounter.iterate_subencounters():
	    	yield subencounter


