cmake_minimum_required(VERSION 3.22)

enable_testing()

file(GLOB PhysicsBenchmarkSource *.cc)

add_executable(PhysicsBenchmark ${PhysicsBenchmarkSource})

target_link_libraries(PhysicsBenchmark Physics)
target_link_libraries(PhysicsBenchmark FEMUtility)
target_link_libraries(PhysicsBenchmark BenchmarkUtility)