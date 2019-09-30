cmake_minimum_required(VERSION 3.7.2)
project(vision_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/BoundingBox2D.msg"
  "msg/BoundingBox3D.msg"
  "msg/Classification2D.msg"
  "msg/Classification3D.msg"
  "msg/Detection2DArray.msg"
  "msg/Detection2D.msg"
  "msg/Detection3DArray.msg"
  "msg/Detection3D.msg"
  "msg/ObjectHypothesis.msg"
  "msg/ObjectHypothesisWithPose.msg"
  "msg/VisionInfo.msg"
)

rosidl_generate_interfaces(generate_vision_msgs
  ${msg_files}
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(vision_msgs_test test/main.cpp)
  add_dependencies(vision_msgs_test generate_vision_msgs)
  ament_target_dependencies(vision_msgs_test
    geometry_msgs
    std_msgs
    sensor_msgs
  )
  # TODO(sloretz) rosidl_generate_interfaces() should make using generated messages in same project simpler
  target_include_directories(vision_msgs_test PUBLIC
    include
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include)

ament_package()
