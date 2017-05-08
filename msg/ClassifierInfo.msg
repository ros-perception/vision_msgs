# Provides meta-information about a classifier (or detector).
#
# This message serves a similar purpose to sensor_msgs/CameraInfo, but instead
# of being tied to hardware, it represents information about a specific computer
# vision classfier or detector. This information stays constant (or relatively
# constant) over time, and so it is wasteful to send it with each individual
# detection message. By listening to these messages, subscribers will receive
# the context in which published vision messages are to be interpreted.

# Used for sequencing
Header header

# Name of the classifier
String method

# ROS parameter name where the metadata database is stored in XML format.
# The exact information stored in the database is left up to the user.
String database_param

# locally-incremented metadata database version. This counter is incremented
# each time the classifier responds to changes in the metadata database.
int32 database_version