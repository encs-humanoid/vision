#!/usr/bin/python
import facedb
import face_util
import os
import sys

if len(sys.argv) == 2:
    fdb = facedb.FaceDB(sys.argv[1])
    for encounter in fdb.iterate():
	faces_txt = os.path.join(encounter.encounter_folder, "faces.txt")
	print "processing " + faces_txt
	for subencounter in encounter.iterate_subencounters():
	    subencounter.calculate_ensemble_overlap()
	# open new faces.txt to overwrite
	with open(faces_txt, "w") as f:
	    for subencounter in encounter.iterate_subencounters():
		for face in subencounter.faces:
		    #print "\t".join([face.image_file, str(face.encounter_id), str(face.max_overlap), str(face.avg_overlap), str(face.min_overlap)])
		    f.write("\t".join([face.image_file, str(face.encounter_id), str(face.max_overlap), str(face.avg_overlap), str(face.min_overlap)]) + "\n")

else:
    print "Usage: <facedb>"
