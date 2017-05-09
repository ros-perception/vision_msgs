# Defines a 3D classification result.
#
# This result does not contain any position information. It is designed for
# classifiers, which simply provide probabilities given a source image.

Header header

# IDs of elements detected with any nonzero probability.
# Does not have to include all possible object
# ids, the scores for any ids not listed are assumed to be 0.
int64[] ids

# The probabilities or confidence values of the detected elements. Must be
# sorted to correspond with the ids field, above
# 0-1, should sum to 1 or less, must be the same length as ids
float64[] scores

# The 2D data that generated these results (i.e. region proposal cropped out of
# the image). Not required for all detectors, so it may be empty.
sensor_msgs/PointCloud2 source