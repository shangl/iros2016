cmake_minimum_required(VERSION 2.8.3)
project(iis_fast_ik)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  message_runtime
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning_interface rospy std_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/kinematics.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(kinematics_tests src/kinematics_tests.cpp)
add_dependencies(kinematics_tests ${PROJECT_NAME})
target_link_libraries(kinematics_tests ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(ik_tests_with_visualization src/ik_tests_with_visualization.cpp)
add_dependencies(ik_tests_with_visualization ${PROJECT_NAME})
target_link_libraries(ik_tests_with_visualization ${PROJECT_NAME} ${catkin_LIBRARIES})
