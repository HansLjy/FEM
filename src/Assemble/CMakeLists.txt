cmake_minimum_required(VERSION 3.22)

file(GLOB_RECURSE ASSEMBLE_SOURCE
        TimeStepper/*.cc
        Target/*.cc
		Data/*.cc
		Objects/*.cc
        )

list(APPEND ASSEMBLE_SOURCE Object.cc System.cc TimeStepper.cc)

add_library(Assemble ${ASSEMBLE_SOURCE})
target_include_directories(Assemble PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Assemble Physics)
target_link_libraries(Assemble Numeric)
target_link_libraries(Assemble Geometry)
target_link_libraries(Assemble FEMUtility)