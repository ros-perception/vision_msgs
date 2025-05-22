cmake_minimum_required(VERSION 3.5)
project(vision_msgs_rviz_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(yaml_cpp_vendor REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(vision_msgs REQUIRED)


## This setting causes Qt's "MOC" generation to happen automatically, which is required for
## Qt's signal/slot connections to work.
set(CMAKE_AUTOMOC ON)


set(vision_msgs_rviz_plugins_headers_to_moc
  include/vision_msgs_rviz_plugins/detection_3d_array.hpp
  include/vision_msgs_rviz_plugins/detection_3d.hpp
  include/vision_msgs_rviz_plugins/bounding_box_3d.hpp
  include/vision_msgs_rviz_plugins/bounding_box_3d_array.hpp
)

foreach(header "${vision_msgs_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(vision_msgs_rviz_plugins_moc_files "${header}")
endforeach()


## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(vision_msgs_rviz_plugins_source_files
  src/detection_3d_array.cpp
  src/detection_3d.cpp
  src/bounding_box_3d.cpp
  src/bounding_box_3d_array.cpp
)


add_library(${PROJECT_NAME} SHARED
  ${vision_msgs_rviz_plugins_moc_files}
  ${vision_msgs_rviz_plugins_source_files}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  rviz_common::rviz_common
)


# Causes the visibility macros to use dllexport rather than dllimport (for Windows, when your plugin should be used as library)
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")


target_link_libraries(${PROJECT_NAME}
  PUBLIC
  rclcpp::rclcpp
  ${vision_msgs_TARGETS}
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
  yaml-cpp::yaml-cpp
)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  vision_msgs
  rviz_common
  rviz_rendering
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY launch conf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

# Install Python executables
install(PROGRAMS
  samples/Detection3DArray.py
  samples/Detection3D.py
  samples/BoundingBox3D.py
  samples/BoundingBox3DArray.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
