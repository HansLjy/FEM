cmake_minimum_required(VERSION 3.22)

add_executable(TreeGenerator TreeGenerator.cpp)

target_link_libraries(TreeGenerator FEMUtility)