cmake_minimum_required(VERSION 2.8.3)
project(uibk_moveit_adapter)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib
    controller_manager
)

find_package(cmake_modules REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp controller_manager actionlib
    DEPENDS
)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

