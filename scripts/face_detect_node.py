from __future__ import division
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
from align_face import CropFace

bridge = CvBridge()

last_ros_image = None
seq = 0

def on_image(ros_image):
    global last_ros_image, seq
    #print "on_image"
    last_ros_image = ros_image
    seq += 1

# given a list of candidate eyes, select the two that are
# most likely the actual eyes
def get_eyes(eyes, image_width):
    if len(eyes) <= 2:
	return eyes
    # the most likely eyes are close to being in line vertically
    # and have one to the left and one to the right of center
    # and are similar in size
    vert = [eye[1] for eye in eyes]
    mean_vert = np.mean(vert)
    for eye in eyes:
	#### TODO Finish this ####
	pass

# given rectangle (x, y, w, h) return the center point (cx, cy)
def get_center(rect):
    (x, y, w, h) = rect
    return (x + w/2, y + h/2)

# rosrun image_transport republish compressed in:=/stereo/left/camera/image _image_transport:=compressed raw out:=/vision/image
# rosrun image_transport republish compressed in:=/camera/image _image_transport:=compressed raw out:=/vision/image
def main(args):
    global last_ros_image, seq
    print "in main"
    face_cascade_path = '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml'
    face_cascade = cv2.CascadeClassifier(face_cascade_path)
    eye_cascade_path = '/usr/share/opencv/haarcascades/haarcascade_eye.xml'
    eye_cascade = cv2.CascadeClassifier(eye_cascade_path)
    nose_cascade_path = '/usr/share/opencv/haarcascades/haarcascade_mcs_nose.xml'
    nose_cascade = cv2.CascadeClassifier(nose_cascade_path)
    mouth_cascade_path = '/usr/share/opencv/haarcascades/haarcascade_mcs_mouth.xml'
    mouth_cascade = cv2.CascadeClassifier(mouth_cascade_path)

    print "register subscriber"
    image_sub = rospy.Subscriber("/vision/image", sensor_msgs.msg.Image, on_image)
    joy_pub = rospy.Publisher("joy", sensor_msgs.msg.Joy, queue_size=1)
    print "init node"
    rospy.init_node('face_detect_node')
    rate = rospy.Rate(30) # 30 Hz

    while not rospy.is_shutdown():
	rate.sleep() # give ROS a chance to run
	if last_ros_image is None:
	    #print "no ros image"
	    pass
	else:
	    #print "imshow"
	    try:
		ros_image = last_ros_image
		#print seq
		last_ros_image = None
		color_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
		#cv2.imshow('Video', color_image)
		gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
		iw, ih = gray_image.shape[1::-1] # note image shape is h, w, d; reverse (h, w)->(w, h)
		for i in xrange(15):
		    size = random.randrange(30, 300)
		    x = random.randrange(0, iw-size)
		    y = random.randrange(0, ih-size)
		    rand_image = gray_image[y:y+size, x:x+size]
		    #cv2.imwrite("images/" + str(ros_image.header.stamp) + "_" + str(i) + ".png", rand_image)
		faces = face_cascade.detectMultiScale(
		    gray_image,
		    scaleFactor=1.2,
		    minNeighbors=5,
		    minSize=(30, 30),
		    flags=cv2.cv.CV_HAAR_SCALE_IMAGE
		)
		# Draw a rectangle around the faces
		for (x, y, w, h) in faces:
		    cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 0, 0), 2)
		    face_only_gray = gray_image[y:y+h, x:x+w]
		    eyes = eye_cascade.detectMultiScale(face_only_gray,
				scaleFactor=1.1,
				minNeighbors=5,
				minSize=(5, 5),
		    		flags=cv2.cv.CV_HAAR_SCALE_IMAGE
		    )
		    for (ex, ey, ew, eh) in eyes:
			cv2.rectangle(color_image, (x+ex, y+ey), (x+ex+ew, y+ey+eh), (0, 255, 0), 2)

		    if len(eyes) == 2:
			#el, er = get_eyes(eyes, face_only_gray.shape[1])
			el_center = get_center(eyes[0])
			er_center = get_center(eyes[1])
			if el_center[0] > er_center[0]:  # swap if left/right are mixed up
				el_center, er_center = er_center, el_center
			image = Image.fromarray(face_only_gray)
			crop = CropFace(image, eye_left=el_center, eye_right=er_center, offset_pct=(0.2,0.2), dest_sz=(30,30))
			cv2.imwrite("images/" + str(ros_image.header.stamp) + "_crop" + str(i) + ".png", np.array(crop))

#		    noses = nose_cascade.detectMultiScale(face_only_gray,
#				scaleFactor=1.1,
#				minNeighbors=5,
#				minSize=(5, 5),
#		    		flags=cv2.cv.CV_HAAR_SCALE_IMAGE
#		    )
#		    for (nx, ny, nw, nh) in noses:
#			cv2.rectangle(color_image, (x+nx, y+ny), (x+nx+nw, y+ny+nh), (255, 0, 0), 2)

#		    mouths = nose_cascade.detectMultiScale(face_only_gray,
#				scaleFactor=1.1,
#				minNeighbors=5,
#				minSize=(10, 10),
#		    		flags=cv2.cv.CV_HAAR_SCALE_IMAGE
#		    )
#		    for (mx, my, mw, mh) in mouths:
#			cv2.rectangle(color_image, (x+mx, y+my), (x+mx+mw, y+my+mh), (0, 0, 255), 2)

		cv2.imshow('Video', color_image)
		joy = sensor_msgs.msg.Joy()
		joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
		if len(faces) > 0:
		    image = SimpleCV.Image(color_image)
		    (x, y, w, h) = faces[0]
		    coords = (x + w/2, y + h/2);
		    joy.axes[0] = 1.5*(image.size()[1] / 2 - coords[0]) / image.size()[1]
		    joy.axes[1] = 1.5*(image.size()[0] / 2 - coords[1]) / image.size()[0]
		    print str(coords) + ": w=" + str(w) + ", h=" + str(h) + ", jx=" + str(joy.axes[0]) + ", jy=" + str(joy.axes[1])
		    joy_pub.publish(joy)
		else:
		    joy = sensor_msgs.msg.Joy()
		    joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		    joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
		    joy_pub.publish(joy)
		cv2.waitKey(1)
	    except CvBridgeError, e:
		print e

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
