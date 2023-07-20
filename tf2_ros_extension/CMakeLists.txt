cmake_minimum_required(VERSION 3.0.2)
project(tf2_ros_extension)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf2_ros)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES tf2_ros_extension
  CATKIN_DEPENDS tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
