cmake_minimum_required(VERSION 2.8.3)
project(kitti2pcd)

set(CMAKE_CXX_COMPILER "g++")
set( CMAKE_BUILD_TYPE Release)
set( CMAKE_CXX_FLAGS "-O3 -std=c++11" )

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
roscpp)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES liebao_pp
 CATKIN_DEPENDS roscpp 
#  DEPENDS system_lib
)

include_directories(
   ${catkin_INCLUDE_DIRS}
   include
   ${PCL_INCLUDE_DIRS}
# ${catkin_INCLUDE_DIRS}
)



add_executable(kitti2pcd
  src/kitti2pcd.cpp
)
add_dependencies(kitti2pcd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kitti2pcd
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
