cmake_minimum_required(VERSION 3.0.2)
project(tftree_merger)


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

add_executable(tftree_merger src/tftree_merger.cpp)

target_link_libraries(tftree_merger
  ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

install(TARGETS tftree_merger
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
