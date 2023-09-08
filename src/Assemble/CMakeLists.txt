cmake_minimum_required(VERSION 3.22)

file(GLOB_RECURSE ASSEMBLE_SOURCE
        TimeStepper/*.cc
		System/*.cc
        Target/*.cc
		Data/*.cc
		Objects/*.cc
        )

list(APPEND ASSEMBLE_SOURCE Object.cc Player.cc Simulator.cc)

add_library(Assemble ${ASSEMBLE_SOURCE})
target_include_directories(Assemble PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Assemble PUBLIC Physics)
target_link_libraries(Assemble PUBLIC Numeric)
target_link_libraries(Assemble PUBLIC FileIO)
target_link_libraries(Assemble PUBLIC Geometry)
target_link_libraries(Assemble PUBLIC FEMUtility)
target_link_libraries(Assemble PUBLIC Renderer)