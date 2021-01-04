cmake_minimum_required(VERSION 2.8.3)
project(managment)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  roscpp
  rospy
  std_msgs
  genmsg
  pcl_conversions
  pcl_ros
  image_transport
  message_generation
  actionlib
  actionlib_msgs 
  sepanta_msgs
)

find_package(Boost COMPONENTS system thread date_time REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



catkin_package(
    INCLUDE_DIRS include 
    CATKIN_DEPENDS message_runtime
)


include_directories(include

  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

include_directories(include  ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(managment src/managment.cpp)
target_link_libraries(managment ${catkin_LIBRARIES} ${Boost_LIBRARIES}  )
add_dependencies(managment sepanta_msgs_generate_messages_cpp)








