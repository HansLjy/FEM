cmake_minimum_required(VERSION 3.22)

project(NumericMethod)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB_RECURSE NUMERIC_SOURCE
        Target.cc
        Constraint/*.cc
        Optimizer/*.cc
        Integrator/*.cc)
add_library(Numeric ${NUMERIC_SOURCE})
target_include_directories(Numeric PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Numeric PRIVATE Physics)
target_link_libraries(Numeric PRIVATE FEMUtility)
target_link_libraries(Numeric PRIVATE Assemble)

if(BUILD_TEST)
    add_subdirectory(Test)
endif()