cmake_minimum_required(VERSION 3.22)

file(GLOB GEOMETRY_TEST_SOURCE 
	CubicSolverTest.cc
	DistanceFunctionTest.cc
	SpatialHashingTest.cc
)

if (GEOMETRY_TEST_SOURCE)
    enable_testing()
    add_executable(GeometryTest ${GEOMETRY_TEST_SOURCE})
    target_link_libraries(GeometryTest PRIVATE FEMUtility)
    target_link_libraries(GeometryTest PRIVATE Geometry)
    include(GoogleTest)
    gtest_discover_tests(GeometryTest)
endif ()

add_executable(VoxelizerTest VoxelizerTest.cc)
target_link_libraries(VoxelizerTest PRIVATE Geometry)
target_link_libraries(VoxelizerTest PRIVATE Renderer)
target_link_libraries(VoxelizerTest PRIVATE FileIO)

add_definitions(-DGEOMETRY_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config")
add_definitions(-DGEOMETRY_TEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data")
