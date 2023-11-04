cmake_minimum_required(VERSION 3.22)

CPMAddPackage(
	cyCodeBase
	GIT_REPOSITORY git@github.com:cemyuksel/cyCodeBase.git
	GIT_TAG ede026f5c1976faa90709690a4290161ed05c19b
)

CPMAddPackage(
	collisiondetection
	GIT_REPOSITORY git@github.com:evouga/collisiondetection.git
	GIT_TAG e5fe5c9767207df5047e375fb20180a665ae186f
	PATCH_COMMAND git am "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/collision-detection.patch"
)

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
target_link_libraries(Geometry PRIVATE collisiondetection)
target_include_directories(Geometry PRIVATE ${cyCodeBase_SOURCE_DIR})
