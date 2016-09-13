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

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moving_det
#  CATKIN_DEPENDS image_geometry laser_geometry pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories( ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Add cmake target dependencies of the library
# add_dependencies(moving_det ${${PROJECT_NAME}_EXPORTED_TARGETS} )

## Declare a C++ executable
 add_executable(moving_obj src/moving_obj.cpp)
 target_link_libraries(moving_obj  ${catkin_LIBRARIES})

add_executable(icp src/icp.cpp)
target_link_libraries(icp ${catkin_LIBRARIES})

add_executable(incremental_icp src/incremental_icp.cpp)
target_link_libraries(incremental_icp  ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(urg_cloud_pub src/urg_cloud_pub.cpp)
target_link_libraries(urg_cloud_pub ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(normal_NDT src/normal_NDT.cpp)
target_link_libraries(normal_NDT ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(shot  src/shot.cpp)
target_link_libraries(shot ${catkin_LIBRARIES} ${PCL_LIBRARIES})
