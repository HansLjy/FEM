cmake_minimum_required(VERSION 3.22)

CPMAddPackage(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
)

add_library(TestUtility FiniteDifference.cpp)
target_link_libraries(TestUtility PRIVATE FEMUtility)
target_link_libraries(TestUtility PUBLIC GTest::gtest_main)
target_include_directories(TestUtility PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})