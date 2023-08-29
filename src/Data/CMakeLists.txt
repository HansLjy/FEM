cmake_minimum_required(VERSION 3.22)

add_executable(TreeGenerator TreeGenerator.cc)
add_executable(MassSpringGenerator MassSpringGenerator.cc)

target_link_libraries(TreeGenerator FEMUtility)