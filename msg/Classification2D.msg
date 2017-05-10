# Defines a 2D classification result.
#
# This result does not contain any position information. It is designed for
# classifiers, which simply provide probabilities given a source image.

Header header

# Class probabilities
CategoryDistribution results

# The 2D data that generated these results (i.e. region proposal cropped out of
# the image). Not required for all use cases, so it may be empty.
sensor_msgs/Image source_img