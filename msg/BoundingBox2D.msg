# Note: This message plans to move to geometry_msgs in the final implementation.

# The 2D position (in pixels) and orientation of the bounding box center.
geometry_msgs/Pose2D center

# The size (in pixels) of the bounding box surrounding the object relative 
# to the pose of its center
uint32 size_x
uint32 size_y
