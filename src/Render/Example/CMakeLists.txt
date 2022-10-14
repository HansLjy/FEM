cmake_minimum_required(VERSION 3.22)

add_executable(RendererDemo ExampleApp.cc main.cc)
target_link_libraries(RendererDemo Renderer)