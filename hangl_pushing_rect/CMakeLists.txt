cmake_minimum_required(VERSION 2.8.3)
project(hangl_pushing_rect)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCPP11SUPPORTED)
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCPP11SUPPORTED)
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS  
  tf
  tf2
  rviz
  roscpp
  kukadu
  pcl_ros
  iis_vrep
  nav_msgs
  std_msgs
  iis_robot
  iis_fast_ik
  sensor_msgs
  moveit_core
  moveit_msgs
  geometry_msgs
  iis_robot_dep
  pcl_conversions
  lwr_description
  uibk_planning_node
  hangl_vision
  moveit_ros_planning_interface
)

find_package(PCL REQUIRED)

add_definitions(-pthread)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES hanglpushinglib
)

AUX_SOURCE_DIRECTORY(src/ PUSHINGSRC)

list(REMOVE_ITEM PUSHINGSRC src/push.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(hanglpushinglib ${PUSHINGSRC})

target_link_libraries(hanglpushinglib kukadu prosim komo ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(hanglpush src/push.cpp)
target_link_libraries(hanglpush kukadu hanglpushinglib prosim komo ${PCL_LIBRARIES} ${catkin_LIBRARIES})
