# Defines a 3D detection result.
#
# This extends a basic 3D classification by including position information,
# allowing a classification result for a specific position in an image to
# to be located in the larger image.

Header header

# Class probabilities
CategoryDistribution results

# The x/y position and (optional) rotation of the bounding box center.
geometry_msgs/Pose2D pose

# (Optional) The size of the bounding box surrounding the object. The center of
# the bounding box is the position of the pose.
float32 bbox_size_x
float32 bbox_size_y