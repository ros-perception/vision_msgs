# Note: This message plans to move to geometry_msgs in the final implementation.

# The x/y position and (optional) rotation of the bounding box center.
geometry_msgs/Pose2D pose

# (Optional) The size of the bounding box surrounding the object. The center of
# the bounding box is the position of the pose.
float32 bbox_size_x
float32 bbox_size_y