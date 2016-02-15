#!/usr/bin/python
from face_util import get_bits
import math
import random
import sys

m = {}
files = []
for f in sys.argv[1:]:
    m[f] = get_bits(f)
    files.append(f)

k = 4

# select random initial cluster centroids
c = []  # value is index of a file
for i in range(k):
    while True:
	index = random.randint(0, len(files) - 1)
	if index not in c:
	    break
    c.append(index)

# calculate distance from each point to each centroid
def distance(b1, b2):
    return -250 * math.log(len(b1 & b2) / 900)

clust = []  # parallel array with files; value is cluster index
# assign point to the nearest centroid


# calculate new centroids ????  HOW?


