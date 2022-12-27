cmake_minimum_required(VERSION 3.22)

file(GLOB GEOMETRY_TEST_SOURCE *.cc)


if (GEOMETRY_TEST_SOURCE)
    enable_testing()
    add_executable(GeometryTest ${GEOMETRY_TEST_SOURCE})
    target_link_libraries(GeometryTest PRIVATE FEMUtility)
    target_link_libraries(GeometryTest PRIVATE Physics)
    target_link_libraries(GeometryTest PRIVATE Geometry)
    include(GoogleTest)
    gtest_discover_tests(GeometryTest)
endif ()

