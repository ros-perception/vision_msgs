# Defines a 2D detection result.
#
# This is similar to a 2D classification, but includes position information,
# allowing a classification result for a specific crop or image point to
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

# The 2D data that generated these results (i.e. region proposal cropped out of
# the image). Not required for all use cases, so it may be empty.
sensor_msgs/Image source_img