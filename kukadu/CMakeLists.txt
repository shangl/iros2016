cmake_minimum_required(VERSION 2.8.3)
project(kukadu)

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

find_package(cmake_modules REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    control_msgs
    eigen_conversions
    geometry_msgs
    iis_robot_dep
    message_generation
    message_runtime
    moveit_core
    moveit_msgs
    moveit_ros_planning_interface
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2_msgs
    tf_conversions
    trajectory_msgs
)

find_package(PCL REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(Armadillo REQUIRED)
find_package(PythonLibs 3.4 REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(PkgConfig REQUIRED)

add_definitions(-fPIC)
add_definitions(-shared)
add_definitions(-pthread)
add_definitions(-DROS_SUPPORT)
add_definitions(-DOSNAME_LINUX=1)

pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
pkg_check_modules(GTK REQUIRED gtkglext-1.0)

catkin_package(
    INCLUDE_DIRS include external/ReflexxesTypeII/include external/komo/share/src /usr/include/python3.4m
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS actionlib_msgs control_msgs control_msgs eigen_conversions
        geometry_msgs iis_robot_dep message_generation moveit_core moveit_msgs
        moveit_ros_planning_interface pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
        tf tf2_msgs tf_conversions trajectory_msgs
    DEPENDS prosim gsl gslcblas ncurses
        stdc++ ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES} ${PythonLibs_LIBRARIES} python3.4m
        komo swift algo gui ors rt lapack blas ann glut GLU GL X11 ${GTK_LIBRARIES} ${GTK2_LIBRARIES} gthread-2.0 qhull m
)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    external/komo/share/src
    /usr/include/python3.4m
    external/ReflexxesTypeII/include
)

set(CURSES_USE_NCURSES TRUE)
set(Python_ADDITIONAL_VERSIONS 3.4)
link_directories(/usr/lib/python3.4/config-3.4m-x86_64-linux-gnu /usr/lib)

set(ENABLE_VISUALIZATION 1)

AUX_SOURCE_DIRECTORY(external/ReflexxesTypeII/src/TypeIIRML/ REFLEXXESSRC)

AUX_SOURCE_DIRECTORY(src/utils/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/types/ KUKADUSRC)
#AUX_SOURCE_DIRECTORY(src/vision/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/trajectory/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/mounted/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/kinematics/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/utils/gnuplot-cpp/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/clustering/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/haptic/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/pushing/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/robotDriver/src/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/utils/easyloggingpp/src/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/metric_learning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/robotDriver/src/kuka/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/kinematics/restriction KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/reinforcement_learning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/reinforcement_learning/base KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/reinforcement_learning/blackjack KUKADUSRC)

add_subdirectory(external/komo)
add_subdirectory(src/learning/projective_simulation)

add_library(${PROJECT_NAME} ${KUKADUSRC})
add_library(reflexxeslib ${REFLEXXESSRC})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES} ${PCL_LIBRARIES}
    reflexxeslib ncurses gsl gslcblas stdc++ pthread dl util m prosim komo swift algo ann gui ors optim motion rt lapack blas glut GLU GL X11 ${GTK_LIBRARIES} ${GTK2_LIBRARIES} gthread-2.0 qhull python3.4m m)

add_executable(kukadu_demo_guided src/demo/guided.cpp)
target_link_libraries(kukadu_demo_guided kukadu ${PCL_LIBRARIES} ${catkin_LIBRARIES} komo swift algo ann gui ors optim motion rt lapack blas glut GLU GL X11 ${GTK_LIBRARIES} ${GTK2_LIBRARIES} gthread-2.0 qhull m)

add_executable(robotino_demo src/demo/robotino_demo.cpp)
target_link_libraries(robotino_demo kukadu ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} python3.4m)
add_dependencies(reflexxeslib ${catkin_EXPORTED_TARGETS})
add_dependencies(kukadu_demo_guided ${catkin_EXPORTED_TARGETS})
add_dependencies(robotino_demo ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
	catkin_add_gtest(kukadutest test/kukadutest.cpp)
endif()

install(TARGETS kukadu_demo_guided robotino_demo RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME} reflexxeslib ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
