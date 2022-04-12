cmake_minimum_required(VERSION 2.8.3)
project(robot_groundstation)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  kinova_msgs
  sensor_msgs
  roscpp
  rospy
  genmsg
  std_msgs
  message_generation
  protobuf_msgs
  cv_bridge
  image_transport
  movo_msgs
  joint_control
  nav_msgs
  geometry_msgs
  )

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ")
set(CMAKE_CXX_FLAGS_RELEASE "-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_DEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DROBOLAND_DEBUG")

find_package(Boost COMPONENTS system thread date_time REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(iir)

pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
      INCLUDE_DIRS include
      CATKIN_DEPENDS message_runtime 
      LIBRARIES robotgroundstation
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${YAML_CPP_INCLUDEDIR} ${Iir_INCLUDEDIR} /user/lib)

#Add executables
add_executable(groundstation_node src/groundstation_node.cpp src/groundstation_ros.cpp src/text.cpp src/DataReduction.cpp  )
target_link_libraries(groundstation_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Iir_LIBRARIES} robogroundstation libiir_static.a)

add_library(robogroundstation src/groundstation.cpp  src/network.cpp)
target_link_libraries(robogroundstation ${catkin_LIBRARIES} zmq )
 
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN ".svn" EXCLUDE
)
