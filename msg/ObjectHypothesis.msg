# An object hypothesis that contains no position information.

# The unique numeric ID of the object class. To get additional information about
#   this ID, such as its human-readable class name, listeners should perform a
#   lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
string id

# The unique ID of object detected if the vision pipeline can identify objects
# across multiple images. Object detection pipelines that do not output such IDs
# should set this to the # empty string "".
# If the two object_id's are the same in different messages for different
# images, it means that the same real object (entity) has been detected in both 
# images. Usefult for trackers.
string object_id


# The probability or confidence value of the detected object. By convention,
#   this value should lie in the range [0-1].
float64 score