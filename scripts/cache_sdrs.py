#!/usr/bin/python
import face_util
import glob
import os
import sys

# pass file names on stdin
# Example:  find facedb -name \*.png | ./cache_sdrs.py
line = raw_input()
count = 1
while line:
    bits = face_util.get_bits(line)
    if count % 100 == 0:
    	print "Finished", count
    count += 1
    line = raw_input()
print "Finished", count
