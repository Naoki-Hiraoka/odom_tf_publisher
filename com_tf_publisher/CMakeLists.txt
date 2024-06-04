cmake_minimum_required(VERSION 3.0.2)
project(com_tf_publisher)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  tf
  tf_conversions
  eigen_conversions
  )
find_package(choreonoid REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES com_tf_publisher
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
 )

add_executable(com_tf_publisher src/com_tf_publisher.cpp)

target_link_libraries(com_tf_publisher
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
 )

#############
## Install ##
#############

install(TARGETS com_tf_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
