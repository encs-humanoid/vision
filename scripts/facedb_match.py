#!/usr/bin/python
import facedb
import face_util
import sys

if len(sys.argv) == 3:
    bits = face_util.get_bits(sys.argv[2])
    fdb = facedb.FaceDB(sys.argv[1])
    for encounter in fdb.iterate():
    	for face in encounter.iterate():
	    print face.image_file, len(bits & face.get_bits())
else:
    print "Usage: <facedb> <file.png>"
