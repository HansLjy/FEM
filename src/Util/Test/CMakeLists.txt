cmake_minimum_required(VERSION 3.22)

enable_testing()
file(GLOB UtilTestSource *.cc)

add_executable(UtilTest GeometricUtilTest.cc)
target_link_libraries(UtilTest FEMUtility)

include(GoogleTest)
gtest_discover_tests(UtilTest)
