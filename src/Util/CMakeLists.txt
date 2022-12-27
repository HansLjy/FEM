cmake_minimum_required(VERSION 3.22)
project(fem-utility)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

# Downlod get-cpm.cmake and use CPM for C++ package management

FetchContent_Declare(
	get_cpm
	URL	https://github.com/cpm-cmake/CPM.cmake/releases/latest/download/get_cpm.cmake
	DOWNLOAD_NO_EXTRACT ON
)

FetchContent_MakeAvailable(
	get_cpm
)

FetchContent_GetProperties(
	get_cpm
	SOURCE_DIR get_cpm_DIR
)

include(${get_cpm_DIR}/get_cpm.cmake)

# Get Eigen for linear algebra

find_package(Eigen3 3.4.0 EXACT)

if(NOT Eigen3_FOUND)
	message("-- Eigen3 not found, download instead")
	CPMAddPackage(
		eigen
		GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3147391d946bb4b6c68edd901f2add6ac1f31f8c
	)
	message("-- Finish downloading Eigen3")
endif()

# find spdlog

find_package(spdlog 1.10.0 EXACT)

if (NOT spdlog_FOUND)
	message("-- spdlog not found, download instead")
	CPMAddPackage(
		spdlog
		GIT_REPOSITORY https://github.com/gabime/spdlog.git
		GIT_TAG 76fb40d95455f249bd70824ecfcae7a8f0930fa3
	)
	message("-- Finish downloading spdlog")
endif()

CPMAddPackage(
	NAME json
	GIT_REPOSITORY https://github.com/ArthurSonzogni/nlohmann_json_cmake_fetchcontent
	GIT_PROGRESS TRUE
	GIT_SHALLOW TRUE
	GIT_TAG v3.11.2
)


set(FEM_UTILITY_SOURCE EigenAll.cc JsonUtil.cc)

if (BUILD_TEST)
	add_subdirectory(TestUtility)
endif()

add_library(FEMUtility ${FEM_UTILITY_SOURCE})
target_include_directories(FEMUtility PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(FEMUtility PUBLIC Eigen3::Eigen)
target_link_libraries(FEMUtility PUBLIC spdlog::spdlog)
target_link_libraries(FEMUtility PUBLIC nlohmann_json::nlohmann_json)
if (BUILD_TEST)
	target_link_libraries(FEMUtility PUBLIC TestUtility)
endif ()

