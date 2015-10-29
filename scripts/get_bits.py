#!/usr/bin/python
from face_util import get_bits
import sys

for f in sys.argv[1:]:
    l = sorted(get_bits(f))
    print f, len(l), l

