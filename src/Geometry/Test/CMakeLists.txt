cmake_minimum_required(VERSION 3.22)

file(GLOB GEOMETRY_TEST_SOURCE 
	CubicSolverTest.cc
	DistanceFunctionTest.cc
	SpatialHashingTest.cc
	VoxelizerTest.cc
	CCDTest.cc
)

if (GEOMETRY_TEST_SOURCE)
    enable_testing()
    add_executable(GeometryTest ${GEOMETRY_TEST_SOURCE})
    target_link_libraries(GeometryTest PRIVATE FEMUtility)
    target_link_libraries(GeometryTest PRIVATE Geometry)
    target_link_libraries(GeometryTest PRIVATE FileIO)
    include(GoogleTest)
    gtest_discover_tests(GeometryTest)
endif ()

add_executable(VoxelizerVisTest VoxelizerVisTest.cc)
target_link_libraries(VoxelizerVisTest PRIVATE Geometry)
target_link_libraries(VoxelizerVisTest PRIVATE Renderer)
target_link_libraries(VoxelizerVisTest PRIVATE FileIO)

add_definitions(-DGEOMETRY_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config")
add_definitions(-DGEOMETRY_TEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data")
