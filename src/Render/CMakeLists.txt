cmake_minimum_required(VERSION 3.22)

project(Renderer)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

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

find_package(OpenGL 4.6 REQUIRED)

find_package(spdlog 1.10.0 EXACT)
if(NOT spdlog_FOUND)
    message("-- spdlog not found, downloading...")
    CPMAddPackage(
        NAME spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG 76fb40d95455f249bd70824ecfcae7a8f0930fa3 # 1.10.0
    )
    message("-- spdlog download success")
endif()

find_package(glm)
if(NOT glm_FOUND)
    message("-- glm not found, downloading...")
    CPMAddPackage(
        NAME glm
        GIT_REPOSITORY https://github.com/Groovounet/glm.git
        GIT_TAG 485700a54648d43ea977c15af73bcf3e56e271be # 0.9.3.4
    )
    message("-- glm download success")
endif()

find_package(Eigen3 3.4.0 EXACT)
if(NOT Eigen3_FOUND)
	message("-- Eigen3 not found, download instead")
	CPMAddPackage (
		eigen
		GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3147391d946bb4b6c68edd901f2add6ac1f31f8c
	)
	message("-- Finish downloading Eigen3")
endif()

add_subdirectory(3rdParty)

add_definitions(-DSHADER_PATH="${CMAKE_CURRENT_SOURCE_DIR}/Shader")

add_library(Renderer Camera/Camera.cc GUI/GUI.cc RenderObject/RendererObject.cc Scene/Scene.cc Shader/Shader.cc)
target_include_directories(Renderer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Renderer PRIVATE OpenGL::GL)
target_link_libraries(Renderer PRIVATE glm::glm)
target_link_libraries(Renderer PUBLIC Eigen3::Eigen)
target_link_libraries(Renderer PRIVATE spdlog::spdlog)
target_link_libraries(Renderer PRIVATE glfw)
target_link_libraries(Renderer PRIVATE glad)
target_link_libraries(Renderer PUBLIC Geometry)
target_link_libraries(Renderer PUBLIC FEMUtility)

if(BUILD_DEMO)
	add_subdirectory(Example)
endif()