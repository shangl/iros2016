cmake_minimum_required(VERSION 2.8.3)
project(hangl_2016_iros)

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
    roscpp
    kukadu
    roscpp
    std_msgs
    iis_fast_ik
    iis_robot_dep
    hangl_pushing_rect
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(PythonLibs 3.4 REQUIRED)

catkin_package(
    CATKIN_DEPENDS kukadu roscpp std_msgs iis_fast_ik iis_robot_dep hangl_pushing_rect
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    /usr/include/python3.4m
)

set(Python_ADDITIONAL_VERSIONS 3.4)
link_directories(/usr/lib/python3.4/config-3.4m-x86_64-linux-gnu /usr/lib)

AUX_SOURCE_DIRECTORY(src/utils IROS2016SRC)
AUX_SOURCE_DIRECTORY(src/controllers IROS2016SRC)

add_definitions(-fPIC)
add_definitions(-shared)
add_definitions(-pthread)
add_definitions(-DROS_SUPPORT)
add_definitions(-DOSNAME_LINUX=1)

add_library(${PROJECT_NAME}
    ${IROS2016SRC}
)

add_executable(iros2016 src/iros2016.cpp)
add_executable(iros2016stat src/iros2016statistics.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME} python3.4m)

target_link_libraries(iros2016 ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME} python3.4m)
target_link_libraries(iros2016stat ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME} python3.4m)

