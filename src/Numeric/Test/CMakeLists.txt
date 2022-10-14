cmake_minimum_required(VERSION 3.22)

file(GLOB NUMERIC_TEST_SOURCE *.cpp *.cc)

enable_testing()
add_executable(NumericTest ${NUMERIC_TEST_SOURCE})

target_link_libraries(NumericTest PRIVATE Numeric)
target_link_libraries(NumericTest PRIVATE FEMUtility)
target_link_libraries(NumericTest PRIVATE Physics)

include(GoogleTest)
gtest_discover_tests(NumericTest)