cmake_minimum_required(VERSION 3.22)

file(GLOB_RECURSE GEOMETRY_SOURCE_FILE
        Render/*.cc
        Collision/*.cc
        Shape/*.cc
		Voxelizer/*.cc
		Embedding/*.cc
		SpatialQuery/*.cc
    )

if(BUILD_TEST)
    add_subdirectory(Test)
endif()

igl_include(restricted triangle)

add_library(Geometry ${GEOMETRY_SOURCE_FILE})
target_include_directories(Geometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Geometry PRIVATE FEMUtility)
target_link_libraries(Geometry PUBLIC Matrix)
target_link_libraries(Geometry PUBLIC FileIO)
target_link_libraries(Geometry PRIVATE igl_restricted::triangle)
