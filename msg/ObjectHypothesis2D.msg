# ID of element detected.
int64 id

# The probability or confidence value of the detected element. Should be in the
# range 0-1.
float64 score

# the 2D pose of the object hypothesis (rotation optional). This pose should be
# defined as the pose of some fixed reference point on the object, such as the
# geometric center of the bounding box or the center of mass of the object.
# Note that different classes predicted
# for the same input data may have different predicted 6D poses.
geometry_msgs/Pose2D pose