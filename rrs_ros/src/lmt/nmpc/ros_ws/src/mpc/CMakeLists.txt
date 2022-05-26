# require a cmake that supports c++14 std, since osqpEigen will use it. Here we set to
# the latest cmake version 3.18.3

# if we do not use osqpEigen, set required version back to 2.8.3
cmake_minimum_required(VERSION 2.8.3)
project(mpc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#set(ENV{PKG_CONFIG_PATH} "$ENV{HOME}/local/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(fcl 0.6.0 REQUIRED)
find_package(octomap REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  kinova_msgs
  movo_msgs
  control_msgs
  interactive_markers
  moveit_core
  moveit_visual_tools
  std_srvs
  #tf
  #tf_conversions
  eigen_conversions
  tf2
  tf2_ros
  perception_msgs
  )
FIND_LIBRARY(EXT_LIBRARY nlopt /usr/local/lib/)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
  CATKIN_DEPENDS
    moveit_core
    interactive_markers
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OCTOMAP_INCLUDE_DIRS}
)
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
  )

## Declare a C++ library
add_library(moveitTool
   src/class/moveitTool.cpp
 )
target_link_libraries(moveitTool
  ${catkin_LIBRARIES}
  fcl
)
#moveittool franka
add_library(moveitTool_franka
   src_franka/class/moveitTool_franka.cpp
 )
target_link_libraries(moveitTool_franka
  ${catkin_LIBRARIES}
  fcl
)

add_library(nmpc_nlopt
  src/class/nmpc_nlopt.cpp
  src/class/hessianCal.cpp
  )
target_link_libraries(nmpc_nlopt
  ${EXT_LIBRARY}
  moveitTool
)

add_library(nmpc_nlopt_real
  src/class/nmpc_nlopt_real.cpp
  src/class/hessianCal.cpp
  )
target_link_libraries(nmpc_nlopt_real
  ${EXT_LIBRARY}
  moveitTool
)

add_library(nmpc_nlopt_movo_pc_real
  src/class/nmpc_nlopt_movo_pc_real.cpp
  src/class/hessianCal.cpp
)
target_link_libraries(nmpc_nlopt_movo_pc_real
  ${EXT_LIBRARY}
  moveitTool
)

#franka nmpc
add_library(nmpc_nlopt_sc_franka
  src_franka/class/nmpc_nlopt_sc_franka.cpp
  src_franka/class/hessianCal_franka.cpp
)
target_link_libraries(nmpc_nlopt_sc_franka
  ${EXT_LIBRARY}
  moveitTool_franka
)

add_library(nmpc_nlopt_movo_sim
  src/class/nmpc_nlopt_movo_sim.cpp
  src/class/hessianCal.cpp
)
target_link_libraries(nmpc_nlopt_movo_sim
  ${EXT_LIBRARY}
  moveitTool
)


# real robot test
add_executable(VelocitySender src/real_robot_test/VelocitySender.cpp)
target_link_libraries(VelocitySender
  ${catkin_LIBRARIES}
)


add_executable(main_nlopt_mpc src/main_nlopt_mpc.cpp)
add_executable(main_real src/main_real.cpp)
target_link_libraries(main_nlopt_mpc
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  nmpc_nlopt
)
target_link_libraries(main_real
  ${catkin_LIBRARIES}
  nmpc_nlopt_real
)
#franka
add_executable(main_sc src_franka/main_sc_franka.cpp)
target_link_libraries(main_sc
  ${catkin_LIBRARIES}
  nmpc_nlopt_sc_franka
)



#for movo system in sim or real 
add_executable(main_movo_pc_real src/main_movo.cpp)
add_executable(main_movo_sc src/main_movo.cpp)
target_link_libraries(main_movo_pc_real
  ${catkin_LIBRARIES}
  nmpc_nlopt_movo_pc_real
)
target_link_libraries(main_movo_sc
  ${catkin_LIBRARIES}
  nmpc_nlopt_movo_sim
)

add_executable(set_marker_goal src/set_marker_goal.cpp)
target_link_libraries(set_marker_goal
  ${catkin_LIBRARIES}
)

add_executable(set_marker_goal_single src/set_marker_goal_single_arm.cpp)
target_link_libraries(set_marker_goal_single
  ${catkin_LIBRARIES}
)




# test on multi threading
#add_executable(testMultiThread src/test/testMultiThread.cpp)
#target_link_libraries(testMultiThread
#  ${catkin_LIBRARIES}
#)

# test on nlopt
#add_executable(nlopt_test src/test/nlopt_test.cpp)
#target_link_libraries(nlopt_test
#  ${catkin_LIBRARIES}
#  ${EXT_LIBRARY}
#)


# test on the method based on OSQP library

# method based on OSQP library

#find_package(osqp REQUIRED)
#find_package(OsqpEigen)
#add_library(nmpc_sqp
#  src/class/nmpc_sqp.cpp
#  src/class/hessianCal.cpp
#  )
#target_link_libraries(nmpc_sqp
#  moveitTool
#  OsqpEigen::OsqpEigen
#)


#add_executable(osqptest src/test/testOSQP.cpp)
#add_executable(osqptest1 src/test/testOSQP1.cpp)
#add_executable(main_sqp_mpc src/main_sqp_mpc.cpp)
#target_link_libraries(main_sqp_mpc
#  ${catkin_LIBRARIES}
#  nmpc_sqp
#)
#target_link_libraries(osqptest
#  ${catkin_LIBRARIES}
#  OsqpEigen::OsqpEigen
#)
#target_link_libraries(osqptest1
#  OsqpEigen::OsqpEigen
#)
#target_link_libraries(kinova_test ${catkin_LIBRARIES})



