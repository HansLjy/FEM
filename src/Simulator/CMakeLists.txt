cmake_minimum_required(VERSION 3.22)

add_library(Simulator Simulator.cpp)
target_link_libraries(Simulator PRIVATE Physics)
target_link_libraries(Simulator PRIVATE Renderer)
target_link_libraries(Simulator PRIVATE Numeric)
target_link_libraries(Simulator PRIVATE FEMUtility)
target_include_directories(Simulator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})