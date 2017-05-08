# Defines a 2D detection result.
#
# This extends a basic 2D classification by including position information,
# allowing a classification result for a specific position in an image to
# to be located in the larger image.

# non-geometric results of detection.
Classification2D classification

# The x/y position and (optional) rotation of the bounding box.
geometry_msgs/Pose2D pose

# The size of the bounding box surrounding the object. The center of the 
# bounding box is the position of the detection point.
geometry_msgs/Vector2 bbox_size