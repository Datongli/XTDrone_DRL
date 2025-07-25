cmake_minimum_required(VERSION 2.8.3)
project(pose_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )

find_package(catkin REQUIRED COMPONENTS
  #armadillo
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_utils
  CATKIN_DEPENDS roscpp
#  CATKIN_DEPENDS geometry_msgs nav_msgs
#  DEPENDS system_lib
)

find_package(Armadillo REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
    include
)

add_library(pose_utils src/pose_utils.cpp)

target_link_libraries(pose_utils
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
)

install(TARGETS pose_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
