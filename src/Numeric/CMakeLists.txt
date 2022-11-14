cmake_minimum_required(VERSION 3.22)

project(NumericMethod)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB_RECURSE NUMERIC_SOURCE
        Constraint/*.cpp
        Optimizer/*.cpp
        Integrator/*.cpp)
add_library(Numeric ${NUMERIC_SOURCE})
target_include_directories(Numeric PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Numeric PRIVATE Physics)
target_link_libraries(Numeric PRIVATE FEMUtility)

if(BUILD_TEST)
    add_subdirectory(Test)
endif()