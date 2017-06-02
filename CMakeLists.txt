cmake_minimum_required(VERSION 3.5)
project(vision_msgs)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
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

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files})
ament_export_dependencies(rosidl_default_runtime)

ament_package()