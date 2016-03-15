cmake_minimum_required(VERSION 2.8.3)
project(iis_robot_dep)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    message_generation
    message_runtime
)

add_message_files(
    FILES
    CartesianImpedance.msg
    FriJointCommand.msg
    FriJointImpedance.msg
    FriJointState.msg
    FriRobotData.msg
    FriRobotJntData.msg
    KukieError.msg
    TactileMatrix.msg
    TactileSensor.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
