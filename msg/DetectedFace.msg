std_msgs/Header header	# use frame_id as identifier for the camera (left or right)
uint32 track_id		# tracking index
uint32[] track_color	# BGR color values for displaying the track
uint32 x		# the x-axis image location of the face in the original image
uint32 y		# the y-axis image location of the face (y increases from top to bottom) in the original image
uint32 w		# the width of the face image
uint32 h		# the height of the face image
uint32 left_eye_x		# the x-axis image location of the left eye relative to the face (x, y)
uint32 left_eye_y		# the y-axis image location of the left eye
uint32 left_eye_w		# the width of the left eye
uint32 left_eye_h		# the height of the left eye
uint32 right_eye_x		# the x-axis image location of the right eye relative to the face (x, y)
uint32 right_eye_y		# the y-axis image location of the right eye
uint32 right_eye_w		# the width of the right eye
uint32 right_eye_h		# the height of the right eye
sensor_msgs/Image image		# a segmented grayscale image of the face (rectangle subimage)
