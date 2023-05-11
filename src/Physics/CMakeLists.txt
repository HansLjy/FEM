cmake_minimum_required(VERSION 3.22)

file(GLOB_RECURSE PHYSICS_SOURCE_FILE
        ExternalForce/*.cc
		Object/*.cc
        )
list(APPEND PHYSICS_SOURCE_FILE
    Object.cc
    DecomposedObject.cc
    ReducedObject.cc
    SampledObject.cc
)

add_library(Physics ${PHYSICS_SOURCE_FILE})

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_link_libraries(Physics PUBLIC OpenMP::OpenMP_CXX)
endif()

target_include_directories(Physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Physics PRIVATE FEMUtility)
target_link_libraries(Physics PRIVATE Geometry)
target_link_libraries(Physics PRIVATE Numeric)
if(BUILD_TEST)
    add_subdirectory(Test)
    add_definitions(-DBUILD_TEST)
endif()