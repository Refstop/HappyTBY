cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} [OpenCV.cmake PATH])

project(opencv_line_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  cv_bridge
  image_transport
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(OpenCV_INCLUDE_DIRS "/usr/include/opencv4;/usr/include")

find_package(OpenCV 4 REQUIRED)
 

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES opencv_line_detection
  CATKIN_DEPENDS rospy roscpp std_msgs
  # DEPENDS system_lib
  DEPENDS OpenCV 4
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIB_DIR})
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/opencv_line_detection.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

##add_executable(AA_code src/temp.cpp)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_executable(ros_opencv_line_detection_node src/opencv_line_detect.cpp)
add_dependencies(ros_opencv_line_detection_node ${ros_opencv_line_detection_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_opencv_line_detection_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
add_executable(overtaking_node src/overtaking.cpp)
add_dependencies(overtaking_node ${overtaking_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(overtaking_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/traffic_light.py scripts/traffic_sign.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




