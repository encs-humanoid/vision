#!/usr/bin/python
import face_util
import glob
import os
import sys

if len(sys.argv) == 3:
    bits = face_util.get_bits(sys.argv[2])
    for png_file in glob.iglob(os.path.join(sys.argv[1], "*.png")):
	print len(bits & face_util.get_bits(png_file)), png_file
else:
    print "Usage: <folder> <file.png>"
