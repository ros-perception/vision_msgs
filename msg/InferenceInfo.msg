# Provides meta-information about a visual pipeline.
#
# This message serves a similar purpose to sensor_msgs/CameraInfo, but instead
#   of being tied to hardware, it represents information about a specific
#   computer vision pipeline. This information stays constant (or relatively
#   constant) over time, and so it is wasteful to send it with each individual
#   result. By listening to these messages, subscribers will receive
#   the context in which published vision messages are to be interpreted.
# Each vision pipeline should publish its VisionInfo messages to its own topic,
#   in a manner similar to CameraInfo.

# Used for sequencing
std_msgs/Header header

# An array of uint16 keys and string values containing the association
#   between class identifiers and their names. According to the amount
#   of classes and the datatype used to store their ids internally, the
#   maxiumum class id allowed (65535 for uint16 and 255 for uint8) belongs to
#   the "UNLABELED" class.
vision_msgs/VisionClass[] class_map 

# The value between 0-1 used as confidence threshold for the inference.
float32 threshold
