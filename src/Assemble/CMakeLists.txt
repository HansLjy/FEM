cmake_minimum_required(VERSION 3.22)

file(GLOB_RECURSE ASSEMBLE_SOURCE
        TimeStepper/*.cc
		System/*.cc
        Assembler/*.cc
		Data/*.cc
)

list(APPEND ASSEMBLE_SOURCE Player.cc Simulator.cc)

add_library(Assemble ${ASSEMBLE_SOURCE})
target_include_directories(Assemble PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Assemble PUBLIC Physics)
target_link_libraries(Assemble PUBLIC Numeric)
target_link_libraries(Assemble PUBLIC FileIO)
target_link_libraries(Assemble PUBLIC Geometry)
target_link_libraries(Assemble PUBLIC Renderer)

if(BUILD_TEST)
	add_subdirectory(Test)
endif(BUILD_TEST)

file(GLOB_RECURSE OBJECT_SOURCE Objects/*.cc)
add_library(Objects OBJECT ${OBJECT_SOURCE})
target_link_libraries(Objects Assemble)
