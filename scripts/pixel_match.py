#!/usr/bin/python
import face_util
import glob
import Image
import os
import sys

if len(sys.argv) == 3:
    test_png_file = sys.argv[2]
    bits = face_util.get_bits(test_png_file)
    image = Image.open(test_png_file)
    for png_file in glob.iglob(os.path.join(sys.argv[1], "*.png")):
    	match = bits & face_util.get_bits(png_file)
	image2 = Image.open(png_file)
	combined = Image.new("RGB", (60, 60))
	combined.paste(image, (0, 0))
	combined.paste(image2, (30, 0))

	# black out all pixels not matched
	pix1 = image.load()
	pix2 = image2.load()
	pix = combined.load()

	for bit in match:
	    index = bit // 16
	    x = index % 30
	    y = index // 30
	    for i in range(-1, 2):
	        for j in range(-1, 2):
		    if x + i >= 0 and x + i < 30 and y + j >= 0 and y + j < 30:
			p1 = pix1[x + i, y + j]
			pix[x + i, 30 + y + j] = (p1, p1, p1)
			p2 = pix2[x + i, y + j]
			pix[30 + x + i, 30 + y + j] = (p2, p2, p2)

	count = 0
	while True:
	    filename = "review/" + str(len(match)) + "-" + str(count) + ".png"
	    if os.path.exists(filename):
	    	count += 1
	    else:
		combined.save(filename)
		break

	print len(match), png_file
else:
    print "Usage: <folder> <file.png>"
