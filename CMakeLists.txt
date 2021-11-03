cmake_minimum_required(VERSION 3.0.2)
project(odom_tf_publisher)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  tf
  tf_conversions
  eigen_conversions
  )
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3)

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
  ${EIGEN_INCLUDE_DIRS}
 )

add_executable(odom_tf_publisher src/odom_tf_publisher.cpp)

target_link_libraries(odom_tf_publisher
  ${catkin_LIBRARIES}
  ${EIGEN_LIBRARIES}
 )

#############
## Install ##
#############

install(TARGETS odom_tf_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
