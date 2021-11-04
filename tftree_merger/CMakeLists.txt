cmake_minimum_required(VERSION 3.0.2)
project(tftree_merger)


set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf
  tf_conversions
  eigen_conversions
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
