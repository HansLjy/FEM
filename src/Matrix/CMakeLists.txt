cmake_minimum_required(VERSION 3.22)

file(GLOB MATRIX_SOURCE *.cc)
add_library(Matrix ${MATRIX_SOURCE})
target_include_directories(Matrix PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Matrix PRIVATE FEMUtility)

if (BUILD_TEST)
    add_subdirectory(Test)
endif()