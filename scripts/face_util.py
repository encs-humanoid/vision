#!/usr/bin/python
#===================================================================
# This is a library of reusable code for face recognition
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================
from __future__ import division
from align_face import CropFace
import csv
import glob
import Image
import numpy as np
import os
import rospy
from scipy import sparse
import vision.msg


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

def get_bits(filename, use_orig=False):
    global cache
    if filename in cache:
	return cache[filename]
    sdr_filename = filename.replace(".png", ".sdr")
    if os.path.exists(sdr_filename):
    	with open(sdr_filename, "r") as f:
	    lines = f.readlines()
	bits = set(eval(lines[0]))
    else:
	image = Image.open(filename)
	cv_image = np.array(image, dtype=np.float32)
	if use_orig:  # use original pixel intensity algorithm
	    bits = set(get_pixelprint_orig(cv_image))
	else:  # use new feature based algorithm
	    bits = set(get_pixelprint(cv_image))
	with open(sdr_filename, "w") as f:
	    f.write(str(sorted(bits)))
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


def create_detected_face_msg(bridge, x, y, w, h, left_eye, right_eye, cv_crop, ros_image):
    detected_face = vision.msg.DetectedFace()
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
    face_image = bridge.cv2_to_imgmsg(cv_crop, encoding="32FC1")
    # copy the header info from the original image to the cutout face image
    face_image.header.seq = ros_image.header.seq
    face_image.header.stamp = ros_image.header.stamp
    face_image.header.frame_id = ros_image.header.frame_id
    detected_face.image = face_image
    detected_face.header.stamp = rospy.Time.now()
    detected_face.header.frame_id = ros_image.header.frame_id
    return detected_face


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
