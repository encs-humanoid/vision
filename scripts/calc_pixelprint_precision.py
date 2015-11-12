#!/usr/bin/python
from __future__ import division
import face_util
import sys


def calc_precision_recall(overlap, threshold, name):
    # threshold the result
    result = [(im, nm, sim) for im, nm, sim in overlap if sim >= threshold]

    true_positives = [1 for _, nm, _ in result if nm == name]
    false_positives = [1 for _, nm, _ in result if nm != name]
    false_negatives = [1 for _, nm, sim in overlap if nm == name and sim < threshold]
    TP = len(true_positives)
    FP = len(false_positives)
    FN = len(false_negatives)

    # calculate precision = TP / (TP + FP)
    if TP > 0 or FP > 0:
	precision = TP / (TP + FP)
    else:
	precision = 0

    # calculate recall = TP / (TP + FN)
    if TP > 0 or FN > 0:
	recall = TP / (TP + FN)
    else:
	recall = 0

    return precision, recall


if __name__ == "__main__":
    if len(sys.argv) >= 2:
	face_names_file = sys.argv[1]
	use_orig = len(sys.argv) > 2

	with open(face_names_file, 'r') as f:
	    lines = f.readlines()

	face_names = [a.split() for a in lines]

	train = [a for a in face_names if "/train/" in a[0]]
	cv = [a for a in face_names if "/cv/" in a[0]]

	print "ImageFile", "Name", "Threshold", "Precision", "Recall"
	for image_file, name in cv:
	    # calculate overlap to all images in train
	    bits = face_util.get_bits(image_file, use_orig)
	    overlap = [(im, nm, len(bits & face_util.get_bits(im, use_orig))) for im, nm in train]

	    for threshold in [50, 100, 150, 200, 250, 300, 350, 400, 450]:
		precision, recall = calc_precision_recall(overlap, threshold, name)

		print image_file, name, str(threshold), str(precision), str(recall)
    else:
	print "Usage: <face_names.txt> [orig]"


