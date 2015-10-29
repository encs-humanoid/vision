#!/usr/bin/python
#===================================================================
# This is a library of reusable code for face recognition
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================
from __future__ import division
import csv
import glob
import Image
import numpy as np
import os
from scipy import sparse


features = np.matrix([
    [1, 1, 0, 1, 1, 0, 0, 0, 0],
    [-1, 1, 0, 1, 1, 0, 0, 0, 0],
    [1, -1, 0, 1, 1, 0, 0, 0, 0],
    [1, 1, 0, -1, 1, 0, 0, 0, 0],
    [1, 1, 0, 1, -1, 0, 0, 0, 0],
    [-1, -1, 0, 1, 1, 0, 0, 0, 0],
    [-1, 1, 0, -1, 1, 0, 0, 0, 0],
    [-1, 1, 0, 1, -1, 0, 0, 0, 0],
    [1, -1, 0, -1, 1, 0, 0, 0, 0],
    [1, -1, 0, 1, -1, 0, 0, 0, 0],
    [1, 1, 0, -1, -1, 0, 0, 0, 0],
    [-1, -1, 0, -1, 1, 0, 0, 0, 0],
    [-1, -1, 0, 1, -1, 0, 0, 0, 0],
    [-1, 1, 0, -1, -1, 0, 0, 0, 0],
    [1, -1, 0, -1, -1, 0, 0, 0, 0],
    [-1, -1, 0, -1, -1, 0, 0, 0, 0],
    [0, 1, 1, 0, 1, 1, 0, 0, 0],
    [0, -1, 1, 0, 1, 1, 0, 0, 0],
    [0, 1, -1, 0, 1, 1, 0, 0, 0],
    [0, 1, 1, 0, -1, 1, 0, 0, 0],
    [0, 1, 1, 0, 1, -1, 0, 0, 0],
    [0, -1, -1, 0, 1, 1, 0, 0, 0],
    [0, -1, 1, 0, -1, 1, 0, 0, 0],
    [0, -1, 1, 0, 1, -1, 0, 0, 0],
    [0, 1, -1, 0, -1, 1, 0, 0, 0],
    [0, 1, -1, 0, 1, -1, 0, 0, 0],
    [0, 1, 1, 0, -1, -1, 0, 0, 0],
    [0, -1, -1, 0, -1, 1, 0, 0, 0],
    [0, -1, -1, 0, 1, -1, 0, 0, 0],
    [0, -1, 1, 0, -1, -1, 0, 0, 0],
    [0, 1, -1, 0, -1, -1, 0, 0, 0],
    [0, -1, -1, 0, -1, -1, 0, 0, 0],
    [0, 0, 0, 1, 1, 0, 1, 1, 0],
    [0, 0, 0, -1, 1, 0, 1, 1, 0],
    [0, 0, 0, 1, -1, 0, 1, 1, 0],
    [0, 0, 0, 1, 1, 0, -1, 1, 0],
    [0, 0, 0, 1, 1, 0, 1, -1, 0],
    [0, 0, 0, -1, -1, 0, 1, 1, 0],
    [0, 0, 0, -1, 1, 0, -1, 1, 0],
    [0, 0, 0, -1, 1, 0, 1, -1, 0],
    [0, 0, 0, 1, -1, 0, -1, 1, 0],
    [0, 0, 0, 1, -1, 0, 1, -1, 0],
    [0, 0, 0, 1, 1, 0, -1, -1, 0],
    [0, 0, 0, -1, -1, 0, -1, 1, 0],
    [0, 0, 0, -1, -1, 0, 1, -1, 0],
    [0, 0, 0, -1, 1, 0, -1, -1, 0],
    [0, 0, 0, 1, -1, 0, -1, -1, 0],
    [0, 0, 0, -1, -1, 0, -1, -1, 0],
    [0, 0, 0, 0, 1, 1, 0, 1, 1],
    [0, 0, 0, 0, -1, 1, 0, 1, 1],
    [0, 0, 0, 0, 1, -1, 0, 1, 1],
    [0, 0, 0, 0, 1, 1, 0, -1, 1],
    [0, 0, 0, 0, 1, 1, 0, 1, -1],
    [0, 0, 0, 0, -1, -1, 0, 1, 1],
    [0, 0, 0, 0, -1, 1, 0, -1, 1],
    [0, 0, 0, 0, -1, 1, 0, 1, -1],
    [0, 0, 0, 0, 1, -1, 0, -1, 1],
    [0, 0, 0, 0, 1, -1, 0, 1, -1],
    [0, 0, 0, 0, 1, 1, 0, -1, -1],
    [0, 0, 0, 0, -1, -1, 0, -1, 1],
    [0, 0, 0, 0, -1, -1, 0, 1, -1],
    [0, 0, 0, 0, -1, 1, 0, -1, -1],
    [0, 0, 0, 0, 1, -1, 0, -1, -1],
    [0, 0, 0, 0, -1, -1, 0, -1, -1]
])

num_features = 16
Sc = None  # lazy initialized
W = None  # lazy initialized
w_dest_rows = []
w_dest_cols = []
v_source_rows = []
a_source_rows = []


def get_pixelprint(cv_crop, bits_per_pixel=50):
    global Sc, W, w_dest_rows, w_dest_cols, v_source_rows, a_source_rows
    cw, ch = cv_crop.shape[1::-1] # note image shape is h, w, d; reverse (h, w)->(w, h)
    v = cv_crop.reshape((cw * ch, 1)).flatten()
    if Sc is None:
    	sc = sparse.lil_matrix((cw*ch, cw*ch))
	for px in xrange(cw):
	    for py in xrange(ch):
	    	row = px*cw + py
		for i in range(-1, 2):
		    for j in range(-1, 2):
			x = px + i
			y = py + j
			s_col = x*cw + y
			if x >= 0 and x < cw and y >= 0 and y < ch:
			    if px > 0 and px < cw-1 and py > 0 and py < ch-1:  # interior
				sc[row, s_col] = 1.0/9.0
			    elif px > 0 and px < cw-1 and (py == 0 or py == ch-1):  # top/bottom edge interior
				sc[row, s_col] = 1.0/6.0
			    elif (px == 0 or px == cw-1) and py > 0 and py < ch-1:  # left/right edge interior
				sc[row, s_col] = 1.0/6.0
			    elif (px == 0 or px == cw-1) and (py == 0 or py == ch-1):  # corner
				sc[row, s_col] = 1.0/4.0
			    w_dest_rows.append((i+1)*3 + (j+1))
			    w_dest_cols.append(row)
			    v_source_rows.append(s_col)
			    a_source_rows.append(row)
	Sc = sc.tocsr()
	W = np.matrix(np.zeros((9, cw * ch)))
    A = Sc.dot(v).flatten()  # compute averages for all pixels
    W[w_dest_rows, w_dest_cols] = v[v_source_rows] - A[a_source_rows]
    M = features * W
    max_indices = np.argmax(M, 0)  # returns index of max value in each col - the top feature for the pixel
    bit_indices = np.array(max_indices % num_features).flatten()
    bits2 = [i * num_features + index for i, index in enumerate(bit_indices)]
    return bits2

# --- ORIGINAL SLOW IMPLEMENTATION
#    bits = []
#    for px in xrange(cw):
#	for py in xrange(ch):
#	    # calculate the average pixel intensity for surrounding pixels
#	    pixels = []
#	    for i in range(-1, 2):
#	    	for j in range(-1, 2):
#		    x = px + i
#		    y = py + j
#		    if x >= 0 and x < cw and y >= 0 and y < ch:
#		    	pixels.append(cv_crop[x][y])
#		    else:
#		    	pixels.append(0)
#
#	    avg_intensity = float(A[px*cw + py])
#
#	    # subtract the average from the original intensities to center the data at the origin
#	    for i in range(len(pixels)):
#	    	if pixels[i] > 0:
#		    pixels[i] -= avg_intensity
#
#	    # match all features to the pixel in all orientations
#	    #match = [np.dot(f, pixels) for f in features]
#	    pixels = np.matrix(pixels)
#	    match = features * pixels.T
#	    	
#	    # select the best matching feature overall
#	    max_feature_index = np.argmax(match) % num_features
#
#	    # set the bit corresponding to that feature to represent the pixel
#	    bits.append((px*ch + py)*num_features + max_feature_index)
#    if bits != bits2:
#    	raise Exception("bits not matched: " + str(bits) + "\n-----\n" + str(bits2))
#    return bits


def get_pixelprint_orig(cv_crop, bits_per_pixel=50):
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


def get_pixels(filename):
    '''
    Return pixels as an array.
    '''
    image = Image.open(filename)
    cv_image = np.array(image, dtype=np.float32)
    cw, ch = cv_image.shape[1::-1] # note image shape is h, w, d; reverse (h, w)->(w, h)
    return np.reshape(cv_image, (1, cw * ch))[0]


def read_y_map(directory, filename):
    with open(os.path.join(directory, filename), "r") as f:
    	lines = f.readlines()
    y_map = {}
    for line in lines:
    	values = line.split()
    	y_map[values[1]] = int(values[0])
    return y_map


def write_dataset(directory):
    y_map = read_y_map(directory, "y-map.txt")

    with open(os.path.join(directory, "X.csv"), "w") as X:
        with open(os.path.join(directory, "y.csv"), "w") as y:
	    X_writer = csv.writer(X)
	    y_writer = csv.writer(y)
	    # find all PNG files under folders in the directory
	    for png_file in glob.iglob(os.path.join(directory, "*", "*.png")):
		# read the image pixels into arrays
		pixels = get_pixels(png_file)

		# write out the X.csv and y.csv files for the dataset
		X_writer.writerow(pixels)
		y_writer.writerow([y_map[png_file.split("/")[-2]]])


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
