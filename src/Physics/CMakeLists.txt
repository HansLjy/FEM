cmake_minimum_required(VERSION 3.22)

file(GLOB_RECURSE PHYSICS_SOURCE_FILE
        Curve/*.cpp
        ExternalForce/*.cpp
        RigidBody/*.cpp
        ReducedObject/*.cpp
        Cloth/*.cpp
        Tree/*.cpp)
list(APPEND PHYSICS_SOURCE_FILE Object.cpp)
list(APPEND PHYSICS_SOURCE_FILE System.cpp)
list(APPEND PHYSICS_SOURCE_FILE Domain.cpp)

add_library(Physics ${PHYSICS_SOURCE_FILE})
target_include_directories(Physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Physics PRIVATE FEMUtility)
target_link_libraries(Physics PRIVATE Geometry)
target_link_libraries(Physics PRIVATE Numeric)

if(BUILD_TEST)
    add_subdirectory(Test)
    add_definitions(-DBUILD_TEST)
endif()