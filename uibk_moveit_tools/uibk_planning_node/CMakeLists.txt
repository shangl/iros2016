cmake_minimum_required(VERSION 2.8.3)
project(uibk_planning_node)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib
    moveit_msgs
    moveit_core
    geometry_msgs
    uibk_moveit_adapter
    uibk_robot_moveit_config
    moveit_ros_planning_interface
)

find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS ${Boost_LIBRARIES}
  CATKIN_DEPENDS moveit_core roscpp actionlib moveit_msgs geometry_msgs uibk_moveit_adapter uibk_robot_moveit_config moveit_ros_planning_interface
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIRS}
)

AUX_SOURCE_DIRECTORY(src/ UIBKPLANNINGSRC)

add_library(${PROJECT_NAME}
    ${UIBKPLANNINGSRC}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_goal src/move_goal.cpp)
target_link_libraries(move_goal ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(kinematics_helper_tests tests/kinematics_helper_tests.cpp)
add_dependencies(kinematics_helper_tests ${PROJECT_NAME})
target_link_libraries(kinematics_helper_tests ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_trajectory_planner tests/test_trajectory_planner.cpp)
add_dependencies(test_trajectory_planner ${PROJECT_NAME})
target_link_libraries(test_trajectory_planner ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_ccontrol tests/test_ccontrol.cpp)
add_dependencies(test_ccontrol ${PROJECT_NAME})
target_link_libraries(test_ccontrol ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
