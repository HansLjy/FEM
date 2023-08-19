cmake_minimum_required(VERSION 3.22)

add_library(Physics INTERFACE)

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_link_libraries(Physics INTERFACE OpenMP::OpenMP_CXX)
endif()

target_include_directories(Physics INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Physics INTERFACE FEMUtility)
if(BUILD_TEST)
    add_subdirectory(Test)
    add_definitions(-DBUILD_TEST)
endif()