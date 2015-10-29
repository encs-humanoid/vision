#!/usr/bin/python
from face_util import overlap
import sys

if len(sys.argv) == 3:
    print overlap(sys.argv, 1, 2)
else:
    print "Usage: file1 file2"
