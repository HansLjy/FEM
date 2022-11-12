cmake_minimum_required(VERSION 3.22)

file(GLOB_RECURSE GEOMETRY_SOURCE_FILE
        CurveShape/*.cpp
        ClothShape/*.cpp
        Mesh/*.cpp
        Rigid/*.cpp
        TreeShape/*.cpp
    )

if(BUILD_TEST)
    add_subdirectory(Test)
endif()

add_library(Geometry ${GEOMETRY_SOURCE_FILE})
target_include_directories(Geometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Geometry PRIVATE FEMUtility)
target_link_libraries(Geometry PRIVATE Physics)