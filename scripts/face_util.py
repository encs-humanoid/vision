#!/usr/bin/python
#===================================================================
# This is a library of reusable code for face recognition
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================
from __future__ import division
import glob
import Image
import numpy as np
import os


def get_pixelprint(cv_crop, bits_per_pixel=50):
    cw, ch = cv_crop.shape[1::-1] # note image shape is h, w, d; reverse (h, w)->(w, h)
    bits = []
    for px in xrange(cw):
	for py in xrange(ch):
	    i = int(max(1, min(int(cv_crop[px][py]), 250) / 5))  # Note:  5 * 50 = 250
	    bits.append((px*ch + py)*bits_per_pixel + i)
    return bits


def overlap(files, i, j):
    return len(get_bits(files[i]) & get_bits(files[j]))


cache = {}

def get_bits(filename):
    global cache
    if filename in cache:
	return cache[filename]
    image = Image.open(filename)
    cv_image = np.array(image, dtype=np.float32)
    bits = set(get_pixelprint(cv_image))
    cache[filename] = bits
    return bits


if __name__ == '__main__':
    # compute average ensemble overlap for face encounters in facedb
    facedb_path = "facedb"
    encounter = ""
    encounters = dict()
    for db_file in glob.iglob(os.path.join(facedb_path, "*", "*.png")):
	paths = db_file.split("/")
	if paths[1] != encounter:
	    encounter = paths[1]
	    encounters[encounter] = []
	encounters[encounter].append(db_file)
	#print db_file

    for encounter in encounters.keys():
	files = encounters[encounter]
	avg_overlap = 0
	cnt_overlap = len(files) * (len(files) - 1) / 2
	min_overlap = None
	max_overlap = 0
	for i in xrange(len(files) - 1):
	    for j in xrange(i +  1, len(files)):
		o = overlap(files, i, j)
		avg_overlap += o / cnt_overlap
		if min_overlap:
		    min_overlap = min(min_overlap, o)
		else:
		    min_overlap = o
		max_overlap = max(max_overlap, o)

	for filename in files:
	    #filename = filename.replace(r'\.\d+\.png|\.png', "." + str(int(avg_overlap)) + ".png")
	    #filename = filename.replace('.png', "." + str(int(avg_overlap)) + ".png")
	    filename = filename.replace('.png', "." + str(int(min_overlap)) + ".png")
	    #print filename
	    # TODO rename file

	print encounter + ": max=" + str(int(max_overlap)) + ", avg=" + str(int(avg_overlap)) + ", min=" +  str(int(min_overlap))
