cmake_minimum_required(VERSION 3.22)

enable_testing()

file(GLOB PhysicsTestSource *.cpp)

add_executable(PhysicsTest ${PhysicsTestSource})

include(GoogleTest)
gtest_discover_tests(PhysicsTest)

target_link_libraries(PhysicsTest Physics)
target_link_libraries(PhysicsTest Numeric)
target_link_libraries(PhysicsTest Geometry)
target_link_libraries(PhysicsTest FEMUtility)