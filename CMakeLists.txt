cmake_minimum_required(VERSION 3.22)

project(FEM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# debug mode
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb -Wall")
set(CMAKE_LINK_LIBRARY_FLAG_DEBUG "-g")

# release mode
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# profile mode
set(CMAKE_CXX_FLAGS_PROFILE "-g -O2")

# C++ 17!!!!
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

mark_as_advanced(
	CMAKE_CXX_FLAGS_PROFILE
)

add_definitions(-DTEXTURE_PATH="${PROJECT_SOURCE_DIR}/data/texture")
add_definitions(-DMODEL_PATH="${PROJECT_SOURCE_DIR}/data/model")
add_definitions(-DOUTPUT_PATH="${PROJECT_SOURCE_DIR}/output")
add_definitions(-DCONFIG_PATH="${PROJECT_SOURCE_DIR}/config")

if(CPU_PARALLEL)
	add_definitions(-DCPU_PARALLEL)
endif()

if(BUILD_TEST)
	add_definitions(-DBUILD_TEST)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "profile")
	message("-- Profile mode")
	add_definitions(-DPROFILE_MODE)
endif()

add_subdirectory(src/Util)
add_subdirectory(src/Numeric)
add_subdirectory(src/Geometry)
add_subdirectory(src/Physics)
add_subdirectory(src/Render)
add_subdirectory(src/Assemble)
add_subdirectory(src/Matrix)
add_subdirectory(src/IO)

add_executable(fem src/main.cc $<TARGET_OBJECTS:Objects>)
target_link_libraries(fem Assemble)

set(BUILD_GENERATOR ON)

if(BUILD_GENERATOR)
    add_subdirectory(src/Data)
endif()