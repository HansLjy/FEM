cmake_minimum_required(VERSION 3.22)

project(FEM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb -Wall")
set(CMAKE_LINK_LIBRARY_FLAG_DEBUG "-g")

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_definitions(-DOUTPUT_PATH="${PROJECT_SOURCE_DIR}/output")
add_definitions(-DCONFIG_PATH="${PROJECT_SOURCE_DIR}/config")
message("-- configuration path: ${PROJECT_SOURCE_DIR}/config")

add_subdirectory(src/Util)
add_subdirectory(src/Numeric)
add_subdirectory(src/Geometry)
add_subdirectory(src/Physics)
add_subdirectory(src/Render)
add_subdirectory(src/Simulator)

add_executable(fem src/main.cpp)
target_link_libraries(fem FEMUtility)
target_link_libraries(fem Numeric)
target_link_libraries(fem Geometry)
target_link_libraries(fem Physics)
target_link_libraries(fem Renderer)
target_link_libraries(fem Simulator)

set(BUILD_GENERATOR ON)

if(BUILD_GENERATOR)
    add_subdirectory(src/Data)
endif()