cmake_minimum_required(VERSION 3.22)

enable_testing()

file(GLOB MatrixTestSource *.cc)

add_executable(MatrixTest ${MatrixTestSource})

include(GoogleTest)
gtest_discover_tests(MatrixTest)

target_link_libraries(MatrixTest FEMUtility)
target_link_libraries(MatrixTest Matrix)