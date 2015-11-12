#!/usr/bin/python
import facedb
import face_util
import os
import pub_face
import rospy
import shutil
from subprocess import call
import sys
import time

if len(sys.argv) == 3:
    face_names_file = sys.argv[1]
    review_folder = sys.argv[2]

    pub_face = pub_face.PubFaceNode("/detected_face", "/target_face", "/unrecognized_face")

    while not rospy.is_shutdown():
	with open(face_names_file, 'r') as f:
	    lines = f.readlines()

	noname = [a.split()[0] for a in lines if len(a.split()) == 1]
	print str(len(noname)) + " unnamed images"

	if len(noname) == 0:
	    sys.exit(0)

	# for first image without a name
	image_file = noname[0]

	# publish image to /detected_face/image
	time.sleep(3)
	pub_face.publish_face(image_file)

	# calculate similarity to other unnamed images
	bits = face_util.get_bits(image_file)
	similarity = [(noname[i], len(bits & face_util.get_bits(noname[i]))) for i in xrange(1, len(noname))]

	# copy images to a folder for review
	threshold = 200
	if not os.path.exists(review_folder):
	    print 'creating directory', review_folder
	    os.mkdir(review_folder)
	for src_image_file, sim in similarity:
	    if sim >= threshold:
		count = 0
		new_image_file = os.path.join(review_folder, str(sim) + '-' + str(count) + '.png')
		while os.path.exists(new_image_file):
		    count += 1
		    new_image_file = os.path.join(review_folder, str(sim) + '-' + str(count) + '.png')
		shutil.copyfile(src_image_file, new_image_file)

	# prompt for name of image
	print "Enter name: "
	name = raw_input()

	if name == "exit":
	    sys.exit(0)

	# prompt for similarity threshold
	print 'Enter match threshold (default ' + str(threshold) + '): '
	threshold_str = raw_input()
	if threshold_str != "":
	    threshold = int(threshold_str)

	# assign name to all images above similarity threshold
	d = dict()
	d[image_file] = name
	for src_image_file, sim in similarity:
	    if sim >= threshold:
		d[src_image_file] = name
	with open(face_names_file, 'w') as f:
	    for line in lines:
		a = line.split()
		if len(a) == 1 and a[0] in d:
		    f.write(a[0] + " " + d[a[0]] + "\n")
		else:
		    f.write(line)

	# clear review folder
	call(["rm " + review_folder + "/*.png"], shell=True)

else:
    print "Usage: <face_names.txt> <review_folder>"
