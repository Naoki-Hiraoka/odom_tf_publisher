cmake_minimum_required(VERSION 3.0.2)
project(base_footprint_publisher)


set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf2_ros
  eigen_conversions
  )

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES odom_tf_publisher
  #  CATKIN_DEPENDS other_catkin_pkg
  #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
 )

add_executable(base_footprint_publisher src/base_footprint_publisher.cpp)

target_link_libraries(base_footprint_publisher
  ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

install(TARGETS base_footprint_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
