cmake_minimum_required(VERSION 3.22)

file(GLOB PHYSICS_SOURCE_FILE
	EnergyModel/*.cc
)
add_library(Physics ${PHYSICS_SOURCE_FILE})

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_link_libraries(Physics PUBLIC OpenMP::OpenMP_CXX)
endif()

target_include_directories(Physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Physics PUBLIC FEMUtility)
if(BUILD_TEST)
    add_subdirectory(Test)
endif()