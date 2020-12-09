cmake_minimum_required(VERSION 2.8.3)
project(tr1cpp)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    roscpp
  LIBRARIES
		tr1cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(tr1cpp
	include/${PROJECT_NAME}/i2c.h
	include/${PROJECT_NAME}/tr1.h
	include/${PROJECT_NAME}/segment.h
	include/${PROJECT_NAME}/joint.h
	src/i2c.cpp
	src/tr1.cpp
	src/arm.cpp
	src/joint.cpp
)
target_link_libraries(tr1cpp ${catkin_LIBRARIES})
