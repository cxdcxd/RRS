cmake_minimum_required(VERSION 3.0)
project(joint_control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
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
  kinova_msgs
  geometry_msgs
  tf_conversions
  robotiq_2f_gripper_control
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

add_message_files(
  FILES
  HapticCommand.msg
  GamepadCommand.msg
  HapticRender.msg
  ArmTrajectory.msg
  ArmTrajectoryList.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS  
  CATKIN_DEPENDS message_runtime
)

catkin_package(
      INCLUDE_DIRS include
      CATKIN_DEPENDS message_runtime 
      LIBRARIES robojointcontrol
)



include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

#Add executables
add_executable(joint_control_node src/joint_control_node.cpp src/joint_control_ros.cpp )
target_link_libraries(joint_control_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} robojointcontrol)

add_library(robojointcontrol src/joint_control.cpp )
target_link_libraries(robojointcontrol ${catkin_LIBRARIES} )
 
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN ".svn" EXCLUDE
)
