cmake_minimum_required(VERSION 2.8.3)
project(movo_hardware_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  pluginlib
  transmission_interface
  urdf
  control_toolbox
  joint_limits_interface
  movo_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    pluginlib
    transmission_interface
    urdf
    control_toolbox
    joint_limits_interface
  LIBRARIES 
    movo_hardware_interface
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(movo_hardware_interface src/movo_hardware_interface.cpp)
target_link_libraries(movo_hardware_interface
	${catkin_LIBRARIES})
add_dependencies(movo_hardware_interface movo_msgs_generate_messages_cpp)

add_executable(movo_hardware_interface_node src/movo_hardware_interface_node.cpp)
add_dependencies(movo_hardware_interface_node movo_msgs_generate_messages_cpp movo_hardware_interface)

target_link_libraries(movo_hardware_interface_node
	movo_hardware_interface
	${catkin_LIBRARIES})
