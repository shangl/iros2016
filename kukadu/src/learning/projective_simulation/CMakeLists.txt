project(projective_simulation)
cmake_minimum_required(VERSION 2.6)

if(${ENABLE_VISUALIZATION})
    add_definitions(-DVISUALIZATION=1)
else()
    add_definitions(-DVISUALIZATION=0)
endif()

find_package(Boost REQUIRED COMPONENTS system)

aux_source_directory(./core SRC_LIST)
aux_source_directory(./utils SRC_LIST)
aux_source_directory(./application SRC_LIST)
aux_source_directory(./visualization SRC_LIST)
aux_source_directory(./application/reward SRC_LIST)

add_library(prosim ${SRC_LIST})

add_executable(necolorranking ./executables/necolorranking.cpp)
add_executable(necolornoranking ./executables/necolornoranking.cpp)
add_executable(evaluatenecolor ./executables/evaluatenecolor.cpp)

if(${ENABLE_VISUALIZATION})
    target_link_libraries(prosim allegro allegro_primitives allegro_font allegro_ttf ${Boost_LIBRARIES})
    target_link_libraries(necolorranking prosim allegro allegro_primitives allegro_font allegro_ttf ${Boost_LIBRARIES})
    target_link_libraries(necolornoranking prosim allegro allegro_primitives allegro_font allegro_ttf ${Boost_LIBRARIES})
    target_link_libraries(evaluatenecolor prosim allegro allegro_primitives allegro_font allegro_ttf ${Boost_LIBRARIES})
else()
    target_link_libraries(prosim ${Boost_LIBRARIES})
    target_link_libraries(necolorranking prosim ${Boost_LIBRARIES})
    target_link_libraries(necolornoranking prosim ${Boost_LIBRARIES})
    target_link_libraries(evaluatenecolor prosim ${Boost_LIBRARIES})
endif()
