# A 3D bounding box that can be positioned and rotated about its center (6 DOF)
# Dimensions of this box are in meters, and as such, it may be migrated to
#   another package, such as geometry_msgs, in the future.

# The 3D position and orientation of the bounding box center
geometry_msgs/Pose center

# The total size (in meters) of the bounding box surrounding the
# object. The X and Y axes are relative to the center pose 
# (including orientation).
geometry_msgs/Vector3 size
