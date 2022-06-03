cmake_minimum_required(VERSION 2.8.3)
project(rrs_ros)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  tf
  genmsg
  std_msgs
  geometry_msgs
  message_generation
  bridge_msgs
  cv_bridge
  sensor_msgs
  message_generation
  pcl_conversions
  pcl_ros
  movo_msgs
  franka_core_msgs
  hardware_interface
  controller_manager
  joint_limits_interface
  control_msgs
  transmission_interface
  dynamic_reconfigure
  )

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ")
set(CMAKE_CXX_FLAGS_RELEASE "-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_DEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Dlmt_DEBUG")

find_package(Boost COMPONENTS system thread date_time REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(Cpd  REQUIRED)
find_package(Fgt  REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

add_message_files(
   DIRECTORY msg FILES PointList.msg
 )

generate_messages(DEPENDENCIES std_msgs geometry_msgs)


generate_dynamic_reconfigure_options(
 reconfigure/Param.cfg
)

catkin_package(
	INCLUDE_DIRS include 
        CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
        LIBRARIES rrsnet 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDEDIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(rrsnet
 
            src/Net1/network.cpp
 	          src/Net2/Share/NTPClient.cpp

            src/Net2/Main/Base/Net2.cpp 
            src/Net2/Main/Base/Net2Base.cpp
            src/Net2/Main/Base/Net2Consul.cpp
            src/Net2/Main/Base/Net2Helper.cpp
            src/Net2/Main/Base/TransmitedData.cpp

            src/Net2/Main/Objects/Net2Publisher.cpp
            src/Net2/Main/Objects/Net2Service.cpp
            src/Net2/Main/Objects/Net2Subscirber.cpp
            src/Net2/Main/Objects/Net2Client.cpp
            src/Net2/Main/Objects/Net2AsyncClient.cpp
           
           )

add_executable(rrs_node 

            src/rrs_node.cpp 
	          src/rrs_ros.cpp 
            src/rrs_net_test.cpp
	    
	    )

add_executable(rrs_cpd_node

            src/rrs_cpd_node.cpp 
	          src/rrs_ros_cpd.cpp 
            src/rrs_net_test.cpp
)	

add_executable(rrs_node_benchmark

            src/rrs_node_benchmark.cpp 
	        src/rrs_ros_benchmark.cpp 
	    
	    )	    

target_link_libraries(rrsnet ${catkin_LIBRARIES} ${YAML_CPP_INCLUDEDIR} zmq ppconsul)

target_link_libraries(rrs_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} rrsnet )
add_dependencies(rrs_node lmt_msgs_generate_messages_cpp config_server_generate_messages_cpp)

target_link_libraries(rrs_cpd_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} rrsnet Cpd::Library-C++ Fgt::Library-C++)
add_dependencies(rrs_cpd_node lmt_msgs_generate_messages_cpp config_server_generate_messages_cpp)

target_link_libraries(rrs_node_benchmark ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} rrsnet )
add_dependencies(rrs_node_benchmark lmt_msgs_generate_messages_cpp config_server_generate_messages_cpp)

add_executable(net2_node src/Net2/Manager/net2_node.cpp src/Net2/Manager/net2_ros.cpp )
target_link_libraries(net2_node ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${YAML_CPP_LIBRARIES} rrsnet )
add_dependencies(net2_node lmt_msgs_generate_messages_cpp config_server_generate_messages_cpp)

add_executable(net2_test_node src/Net2/Test/net2_test_node.cpp src/Net2/Test/net2_test_ros.cpp )
target_link_libraries(net2_test_node ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${YAML_CPP_LIBRARIES} rrsnet )
add_dependencies(net2_test_node lmt_msgs_generate_messages_cpp config_server_generate_messages_cpp)

