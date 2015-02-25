std_msgs/Header header		# use frame_id as identifier for the camera (left or right)
uint32 encounter_id		# id of the face encounter
uint32 max_overlap		# minimum ensemble overlap for the face encounter
uint32 avg_overlap		# average ensemble overlap for the face encounter
uint32 min_overlap		# maximum ensemble overlap for the face encounter
string filepath			# path to the learned image file path in the face database
uint32[] bits			# sparse bit pattern for use in recognition of this face
