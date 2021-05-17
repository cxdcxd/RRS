cmake_minimum_required(VERSION 2.8.3)
project(perception_mpc)
add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  roscpp
  rospy
  std_msgs
  sensor_msgs
  octomap_msgs
  perception_msgs
  tf2_ros
)

catkin_package()
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}  
)


## Declare a cpp executable
#add_executable(pc_server point_cloud_server.cpp)
#find_package(PCL REQUIRED)
#include_directories(
#  ${PCL_INCLUDE_DIRS}
#)
## Specify libraries to link a library or executable target against
#target_link_libraries(pc_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cloud_server point_cloud_server.cpp)
find_package(octomap REQUIRED)
find_package(PCL REQUIRED)
include_directories(
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(cloud_server ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

