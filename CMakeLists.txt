cmake_minimum_required(VERSION 3.10)
project(PathfindingProject VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MINGW)
    add_compile_options(-static)
endif()

set(SOURCES
    src/grid_map.cpp
    src/a_star.cpp
    src/jps.cpp
    src/visualizer.cpp
    src/main.cpp
)

add_executable(pathfinding ${SOURCES})

target_include_directories(pathfinding PRIVATE ${CMAKE_SOURCE_DIR}/include)
