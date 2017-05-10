# Defines a 3D detection result.
#
# This extends a basic 3D classification by including position information,
# allowing a classification result for a specific position in an image to
# to be located in the larger image.

Header header

# Class probabilities
CategoryDistribution results

# The element's pose. This pose should be
# defined as the pose of some fixed reference point on the object, such as the
# geometric center of the bounding box or the center of mass of the object.
# Note that this pose is not stamped, but frame information can be provided by
# setting classification.header.frame_id.
geometry_msgs/Pose pose

# The 3D data that generated these results (i.e. region proposal cropped out of
# the image). Not required for all detectors, so it may be empty.
sensor_msgs/PointCloud2 source_cloud