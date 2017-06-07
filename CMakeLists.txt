cmake_minimum_required(VERSION 2.8.3)
project(vision_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs)

add_message_files(FILES
  BoundingBox2D.msg
  BoundingBox3D.msg
  Classification2D.msg
  Classification3D.msg
  Detection2DArray.msg
  Detection2D.msg
  Detection3DArray.msg
  Detection3D.msg
  ObjectHypothesis.msg
  ObjectHypothesisWithPose.msg
  VisionInfo.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  geometry_msgs
)