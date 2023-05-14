cmake_minimum_required(VERSION 3.22)

add_executable(RendererDemo ExampleApp.cc main.cc)
target_link_libraries(RendererDemo Renderer)
add_definitions(-DRENDERER_TEXTURE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/texture")
