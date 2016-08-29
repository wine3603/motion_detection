cmake_minimum_required(VERSION 2.8.3)
project(moving_det)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_geometry
  laser_geometry
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moving_det
#  CATKIN_DEPENDS image_geometry laser_geometry pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(moving_det
#   src/moving_det.cpp
#)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(moving_det ${${PROJECT_NAME}_EXPORTED_TARGETS} )

## Declare a C++ executable
 add_executable(moving_obj src/moving_obj.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(moving_det_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(moving_obj
   ${catkin_LIBRARIES}
 )


