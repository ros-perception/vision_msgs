# A list of 3D detections, for a multi-object 3D detector.

std_msgs/Header header

# list of the detected proposals. For a multi-proposal detector, this list could
# have many objects.
Detection3d[] detections
