add_library(FileIO FileIO.cc ObjIO.cc)

target_link_libraries(FileIO PUBLIC FEMUtility)
target_include_directories(FileIO PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})