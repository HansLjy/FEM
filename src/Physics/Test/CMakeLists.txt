cmake_minimum_required(VERSION 3.22)

enable_testing()

file(GLOB PhysicsTestSource *.cc)

add_executable(PhysicsTest ${PhysicsTestSource})

include(GoogleTest)
gtest_discover_tests(PhysicsTest)

target_link_libraries(PhysicsTest Physics)
target_link_libraries(PhysicsTest Numeric)
target_link_libraries(PhysicsTest Geometry)
target_link_libraries(PhysicsTest FEMUtility)

add_definitions(-DPHYSICS_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config")
message("-- physics test configuration path ${CMAKE_CURRENT_SOURCE_DIR}/config")