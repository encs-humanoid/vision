from __future__ import division
from cv_bridge import CvBridge, CvBridgeError
from subprocess import call
import cv2
import SimpleCV
import Image
import sys
import random
import numpy as np

def get_fingerprint(image, bits_per_pixel=200, bits_per_value=10):
    cv_crop = np.array(image, dtype=np.float32)
    cw, ch = cv_crop.shape[1::-1] # note image shape is h, w, d; reverse (h, w)->(w, h)
    # for each pixel, 200 bits are allocated
    # given pixel intensity i where i < 190, set bits [i:i+10]
    bits = []
    for px in xrange(cw):
	for py in xrange(ch):
	    i = int(cv_crop[px][py])
	    if i < 0:
		i = 0
	    if i > bits_per_pixel - bits_per_value:
		i = bits_per_pixel - bits_per_value
	    for k in xrange(bits_per_value):
		bits.append(px*py*bits_per_pixel + i + k)
    return bits

class Match:
    def __init__(self, overlap, file):
	self.overlap = overlap
	self.file = file

def main(args):
    if len(args) <= 1:
	print "Usage: " + args[0] + " image_file"
	return
    test_file = args[1]

    test_image = Image.open(test_file)
    
    test_bits = set(get_fingerprint(test_image))

    with open("imagedb.txt") as f:
	lines = f.readlines()

    list = []
    for db_file in [f.strip('\n') for f in lines]:
	db_image = Image.open(db_file)
	db_bits = set(get_fingerprint(db_image))
	overlap = len(test_bits & db_bits)
	list.append(Match(overlap, db_file))

    list.sort(key=lambda x: x.overlap, reverse=True)

    for i in xrange(min(20, len(list))):
	call(["cp", list[i].file, "match"])

if __name__ == '__main__':
    main(sys.argv)
