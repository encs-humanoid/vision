#!/usr/bin/python
from __future__ import division
import argparse
import heapq
import roslib
#roslib.load_manifest('demo')
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import cv2
import SimpleCV
import Image
import sys
import random
import numpy as np
import multiprocessing
import Queue
from align_face import CropFace

bridge = CvBridge()

last_ros_image = None
seq = 0

def on_image(ros_image):
    global last_ros_image, seq
    last_ros_image = ros_image
    seq += 1

face_cascade_path = '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml'
face_cascade = cv2.CascadeClassifier(face_cascade_path)
eye_cascade_path = '/usr/share/opencv/haarcascades/haarcascade_eye.xml'
eye_cascade = cv2.CascadeClassifier(eye_cascade_path)

def find_faces(gray_image):
    faces = face_cascade.detectMultiScale(
	gray_image,
	scaleFactor=1.2,
	minNeighbors=5,
	minSize=(30, 30),
	flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    )
    return faces

def find_eyes(face_only_gray):
    eyes = eye_cascade.detectMultiScale(face_only_gray,
		scaleFactor=1.1,
		minNeighbors=5,
		minSize=(5, 5),
		flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    )
    return eyes

def crop_and_normalize(face_only_gray, eyes, target_intensity=95, target_range=150, offset_pct=(0.2,0.2), dest_sz=(30,30)):
    el_center = get_center(eyes[0])
    er_center = get_center(eyes[1])
    # swap if left/right are mixed up
    if el_center[0] > er_center[0]:
	    el_center, er_center = er_center, el_center

    # Align eyes and crop image
    image = Image.fromarray(face_only_gray)
    crop = CropFace(image, eye_left=el_center, eye_right=er_center, offset_pct=offset_pct, dest_sz=dest_sz)
    cv_crop = np.array(crop, dtype=np.float32)

    # Normalize intensity
    average_intensity = int(np.mean(cv_crop))
    max_intensity = np.max(cv_crop)
    min_intensity = np.min(cv_crop)
    cv_crop = (cv_crop - average_intensity) * (target_range / (max_intensity - min_intensity)) + target_intensity

    return cv_crop

# given rectangle (x, y, w, h) return the center point (cx, cy)
def get_center(rect):
    (x, y, w, h) = rect
    return (x + w/2, y + h/2)

def get_pixelprint(cv_crop, bits_per_pixel=50):
    cw, ch = cv_crop.shape[1::-1] # note image shape is h, w, d; reverse (h, w)->(w, h)
    bits = []
    for px in xrange(cw):
	for py in xrange(ch):
	    i = int(max(1, min(int(cv_crop[px][py]), 250) / 5))  # Note:  5 * 50 = 250
	    #bits.append((px*ch + py)*bits_per_pixel + i-1)
	    bits.append((px*ch + py)*bits_per_pixel + i)
	    #bits.append((px*ch + py)*bits_per_pixel + i+1)
    return bits

class Match:
    def __init__(self, overlap, file, is_last=False):
	self.overlap = overlap
	self.file = file
	self.is_last = is_last

class DBImage:
    def __init__(self, file, fp):
	self.file = file
	self.fp = fp

class Processor(multiprocessing.Process):
    def __init__(self):
	multiprocessing.Process.__init__(self)
	self.db_queue = multiprocessing.Queue()
	self.input_queue = multiprocessing.Queue()
	self.output_queue = multiprocessing.Queue()
	self._has_more = False
	self.image_database = []

    def reset(self):
	self._has_more = True

    def has_more(self):
	return self._has_more

    def finish(self):
	self._has_more = False

    def run(self):
	try:
	    # on startup, read in the portion of the image database allocated to this process
	    while True:
		db_file = self.db_queue.get()
		if db_file == "END":
		    break
		db_image = Image.open(db_file)
		cv_image = np.array(db_image, dtype=np.float32)
		db_bits = set(get_pixelprint(cv_image))
		self.image_database.append(DBImage(db_file, db_bits))

	    # wait for a test recognition to come, process it, and send back the match results
	    while True:
		test_bits = self.input_queue.get()
		count = 0
		for item in self.image_database:
		    overlap = len(item.fp & test_bits)
		    count += 1
		    self.output_queue.put(Match(overlap, item.file, count == len(self.image_database)))
	except KeyboardInterrupt:
	    pass


# rosrun image_transport republish compressed in:=/stereo/left/camera/image _image_transport:=compressed raw out:=/vision/image
# rosrun image_transport republish compressed in:=/camera/image _image_transport:=compressed raw out:=/vision/image
def main(args):
    global last_ros_image, seq

    parser = argparse.ArgumentParser(description="Detect faces in a ROS image stream")
    parser.add_argument("-d", "--dir", dest="output_directory", metavar="DIRECTORY", help="write cropped faces to directory")
    parsed_args = parser.parse_args(args[1:]) # http://stackoverflow.com/questions/17118999/python-argparse-unrecognized-arguments

    num_processors = min(multiprocessing.cpu_count(), 4)
    processors = [Processor() for i in xrange(num_processors)]
    processes = []
    for w in processors:
        processes.append(w)
        w.daemon = False
        w.start()

    with open("imagedb.txt") as f:
	lines = f.readlines()

    index = 0
    for db_file in [f.strip('\n') for f in lines]:
	if index >= len(processors):
	    index = 0
	processors[index].db_queue.put(db_file)
	index += 1

    for w in processors:
	w.db_queue.put("END")

    image_sub = rospy.Subscriber("/vision/image", sensor_msgs.msg.Image, on_image)
    joy_pub = rospy.Publisher("joy", sensor_msgs.msg.Joy, queue_size=1)
    rospy.init_node('face_detect_node')
    rate = rospy.Rate(30) # 30 Hz

    try:
	while not rospy.is_shutdown():
	    rate.sleep() # give ROS a chance to run
	    if last_ros_image is None:
		#print "no ros image"
		pass
	    else:
		try:
		    # grab the last ROS image we have received and clear the
		    # image handle so we don't pick up the same image next time
		    ros_image, last_ros_image = last_ros_image, None

		    # convert the ROS image to OpenCV and convert it to gray scale
		    color_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
		    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

		    faces = find_faces(gray_image)
		    i = 0
		    for (x, y, w, h) in faces:
			# Draw a rectangle around the faces
			cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 0, 255), 2)

			# Extract just the face as a subimage
			face_only_gray = gray_image[y:y+h, x:x+w]

			# look for eyes only in the top of the image
			eyes = find_eyes(face_only_gray[0.2*h:0.6*h, :])
			eyes = [(ex, ey + int(0.2*h), ew, eh) for (ex, ey, ew, eh) in eyes]

			# Draw rectangles around the eyes
			for (ex, ey, ew, eh) in eyes:
			    cv2.rectangle(color_image, (x+ex, y+ey), (x+ex+ew, y+ey+eh), (0, 255, 0), 2)

			if len(eyes) == 2:
			    cv_crop = crop_and_normalize(face_only_gray, eyes)
			    
			    test_bits = set(get_pixelprint(cv_crop))
			    
			    num_top_matches = 5
			    heap = [] # will store heap of tuples (overlap, Match)
			    for processor in processors:
				processor.reset()
				processor.input_queue.put(test_bits)
			    running = True
			    while running:
				running = False
				for processor in processors:
				    if processor.has_more():
					running = True
					try:
					    result = processor.output_queue.get_nowait()
					    if result.is_last:
						processor.finish()
					    item = (result.overlap, result)
					    if len(heap) < num_top_matches or result.overlap > heap[0][0]:
						if len(heap) == num_top_matches:
						    heapq.heappushpop(heap, item)
						else:
						    heapq.heappush(heap, item)
					except Queue.Empty, e:
					    pass

			    list = [item[1] for item in heap]  # note: list is not sorted by overlap

			    # determine which person is most frequently returned in the num_top_matches matches
			    top = dict()
			    person_name = "unknown"
			    for i in xrange(min(num_top_matches, len(list))):
				person_name = list[i].file.split('/')[1]
				print person_name + ", " + str(list[i].overlap)
				if person_name in top:
				    top[person_name] += 1
				else:
				    top[person_name] = 1
			    top_count = 0
			    top_name = None
			    for name in top.keys():
				if top[name] > top_count:
				    top_count = top[name]
				    top_name = name
				    
			    print person_name + " at " + str((x,y)) + ": w=" + str(w) + ", h=" + str(h)

			    # label the face with the recognized name
			    cv2.putText(color_image, top_name, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)

			    if parsed_args.output_directory:
				name = str(ros_image.header.stamp) + "_crop" + str(i)
				cv2.imwrite(parsed_args.output_directory + "/" + name + ".png", cv_crop)

			    i += 1

		    cv2.imshow('Video', color_image)

		    # Face Tracking - generate Joy messages to move face toward the center of view
		    joy = sensor_msgs.msg.Joy()
		    joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		    joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
		    if len(faces) > 0:
			image = SimpleCV.Image(color_image)
			(x, y, w, h) = faces[0]
			coords = (x + w/2, y + h/2);
			scale = 0.5
			joy.axes[0] = scale*(image.size()[1] / 2 - coords[0]) / image.size()[1]
			joy.axes[1] = scale*(image.size()[0] / 2 - coords[1]) / image.size()[0]
			#print str(coords) + ": w=" + str(w) + ", h=" + str(h) + ", jx=" + str(joy.axes[0]) + ", jy=" + str(joy.axes[1])
			joy_pub.publish(joy)
		    else:
			joy = sensor_msgs.msg.Joy()
			joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
			joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
			joy_pub.publish(joy)

		    # Let OpenCV's event loop run so the video will display
		    cv2.waitKey(1)
		except CvBridgeError, e:
		    print e
    except KeyboardInterrupt:
	pass

    for w in processes:
	w.join()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
