from __future__ import division
import roslib
#roslib.load_manifest('demo')
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import SimpleCV
import sys

bridge = CvBridge()

last_ros_image = None
seq = 0

def on_image(ros_image):
    global last_ros_image, seq
    #print "on_image"
    last_ros_image = ros_image
    seq += 1

# rosrun image_transport republish compressed in:=/stereo/left/camera/image _image_transport:=compressed raw out:=/vision/image
# rosrun image_transport republish compressed in:=/camera/image _image_transport:=compressed raw out:=/vision/image
def main(args):
    global last_ros_image, seq
    print "in main"
    cascPath = '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml'
    #cascPath = 'haarcascade_frontalface_default.xml'
    faceCascade = cv2.CascadeClassifier(cascPath)

    print "register subscriber"
    image_sub = rospy.Subscriber("/vision/image", Image, on_image)
    joy_pub = rospy.Publisher("joy", Joy, queue_size=1)
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
		faces = faceCascade.detectMultiScale(
		    gray_image,
		    scaleFactor=1.2,
		    minNeighbors=5,
		    minSize=(30, 30),
		    flags=cv2.cv.CV_HAAR_SCALE_IMAGE
		)
		# Draw a rectangle around the faces
		for (x, y, w, h) in faces:
		    cv2.rectangle(gray_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
		cv2.imshow('Video', gray_image)
		joy = Joy()
		joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
		if len(faces) > 0:
		    image = SimpleCV.Image(color_image)
		    (x, y, w, h) = faces[0]
		    coords = (x + w/2, y + h/2);
		    joy.axes[0] = 1.5*(image.size()[1] / 2 - coords[0]) / image.size()[1]
		    joy.axes[1] = 1.5*(image.size()[0] / 2 - coords[1]) / image.size()[0]
		    print str(coords) + ": ix=" + str(image.size()[1]) + ", iy=" + str(image.size()[0]) + ", jx=" + str(joy.axes[0]) + ", jy=" + str(joy.axes[1])
		    joy_pub.publish(joy)
		else:
		    joy = Joy()
		    joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		    joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
		    joy_pub.publish(joy)
		cv2.waitKey(1)
	    except CvBridgeError, e:
		print e

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
