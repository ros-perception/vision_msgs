# A 2D bounding box without rotation.
# All dimensions are in pixels, and are defined using the same format as OpenCV:
#   http://docs.opencv.org/java/2.4.9/org/opencv/core/Rect.html

# Horizontal position of the upper-left bounding box corner, measured from the
#   left edge of the image.
int32 x

# Vertical position of the upper-left bounding box corner, measured from the
#   top of the image.
int32 y

# Width (x-size) of the bounding box, in pixels.
int32 width

# Height (y-size) of the bounding box, in pixels.
int32 height