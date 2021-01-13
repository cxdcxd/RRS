cmake_minimum_required(VERSION 2.8.3)
project(lmt_movebase)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  roscpp
  rospy
  genmsg
  pcl_conversions
  pcl_ros
  tf
  std_msgs
  nav_msgs
  geometry_msgs
  image_transport
  message_generation
  actionlib
  actionlib_msgs 
  lmt_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ")

find_package(Boost  REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN REQUIRED eigen3)

add_message_files(
   DIRECTORY msg FILES PointList.msg
 )

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
      INCLUDE_DIRS include 
      CATKIN_DEPENDS message_runtime
      CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${SERIAL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(lmt_movebase src/lmt_movebase_node.cpp src/lmt_movebase.cpp)
target_link_libraries(lmt_movebase ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(lmt_movebase lmt_msgs_generate_messages_cpp)

add_executable(client_movebase src/client_movebase.cpp )
target_link_libraries(client_movebase ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(client_movebase lmt_msgs_generate_messages_cpp)






