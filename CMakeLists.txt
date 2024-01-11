cmake_minimum_required(VERSION 3.1)
project(map_update)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++14 -pthread" )


find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  #cv_bridge
  #pcl library
  pcl_conversions
  pcl_ros
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

find_package(PCL 1.10 REQUIRED)


#if(Threads_FOUND)
#    set(CMAKE_CXX_FLAGS  "-pthread")
#endif()

#find_package(PCL REQUIRED QUIET)
set(PCL_INCLUDE_DIR /usr/include/pcl-1.10)


set(EXTERNAL_LIBS "")

set(EXTERNAL_LIBS 
${EXTERNAL_LIBS} 
${Boost_LIBRARIES}
${PCL_LIBRARIES}
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
#  INCLUDE_DIRS include
#  LIBRARIES imlsMatcher
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(map_update_LIOSAM src/map_update_LIOSAM.cpp src/voxel.cpp)

add_executable(map_update src/map_update_single.cpp src/voxel.cpp)

add_executable(debug_map_update src/map_update.cpp src/voxel.cpp)

add_executable(debug_record_data src/record_data.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(map_update_LIOSAM
    ${catkin_LIBRARIES}
    ${EXTERNAL_LIBS}
)

target_link_libraries(map_update
    ${catkin_LIBRARIES}
    ${EXTERNAL_LIBS}
)

target_link_libraries(debug_map_update
    ${catkin_LIBRARIES}
    ${EXTERNAL_LIBS}
)

target_link_libraries(debug_record_data 
    ${catkin_LIBRARIES}
    ${EXTERNAL_LIBS}
)