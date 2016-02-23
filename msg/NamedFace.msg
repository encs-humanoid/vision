std_msgs/Header header	# use frame_id as identifier for the camera (left or right)
uint32 track_id		# tracking index
string[] names		# parallel array of names in confidence order (names[0] is highest confidence)
float32[] confs		# parallel array of confidence values
