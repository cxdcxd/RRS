##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(perception_msgs)
add_compile_options(-std=c++11)
#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -Wno-cpp" )

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs
  message_generation
)

##############################################################################
# Messages, Services and Actions
##############################################################################
add_message_files(
        DIRECTORY msg
        FILES
        Cylinders.msg
)

## Generate services 
#add_service_files(
#)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 CATKIN_DEPENDS message_runtime geometry_msgs
)



#include_directories(
#  ${catkin_INCLUDE_DIRS}
#  include
#  ${roscpp_INCLUDE_DIRS}
#)




