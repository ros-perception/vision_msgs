# A message to contain the output of a Semantic Segmentation process

std_msgs/Header header

uint32 height         # mask image height, that is, number of rows
uint32 width          # mask image width, that is, number of columns

# the bytes of the single-channel image made by
# the segmentation mask
uint8[] data # 

# the confidence of the inference of each pixel
# between 0-100%
uint8[] confidence 

# an array of SemanticClasses specifying which integer value in the
# segmentation mask corresponds to each semantic class
vision_msgs/SemanticClass[] class_map 

# the threshold value used in the segmentation model
float32 threshold