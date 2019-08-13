cmake_minimum_required(VERSION 3.1)
project(vision_msgs_rviz)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Catkin
find_package(catkin REQUIRED
        COMPONENTS
            rviz
            vision_msgs
            eigen_conversions
            
)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS vision_msgs
)

find_package (Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

link_directories(${catkin_LIBRARY_DIRS})

## QT
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)
#set(CMAKE_AUTOMOC ON)

## Etc
include_directories(include)

###########
## Build ##
###########
set(src_files

  #src/bounding_box_array_display.cpp
  src/bounding_box_visual.cpp

  src/detection_3d_array_display.cpp
  src/detection_3d_visual.cpp

  )

include_directories(${CMAKE_CURRENT_BINARY_DIR})
qt_wrap_cpp(${PROJECT_NAME} 
  moc_files 
  #include/vision_msgs_rviz/bounding_box_array_display.h 
  include/vision_msgs_rviz/detection_3d_array_display.h 
  )

add_library(${PROJECT_NAME} ${src_files} ${moc_files})

target_link_libraries(${PROJECT_NAME}
    ${QT_LIBRARIES} ${catkin_LIBRARIES}
 )

#############
## Install ##
#############
# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark config files for installation
install(FILES
        plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

#install(DIRECTORY media/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)


