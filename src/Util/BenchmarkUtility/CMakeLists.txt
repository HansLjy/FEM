cmake_minimum_required(VERSION 3.22)

find_package(benchmark 1.8.3 EXACT REQUIRED)
if (NOT benchmark_FOUND)
	message("-- benchmark not found, download instead")
	set(BENCHMARK_DOWNLOAD_DEPENDENCIES on)
	CPMAddPackage(
		benchmark
		GIT_REPOSITORY git@github.com:google/benchmark.git
		GIT_TAG v1.8.3
	)
	message("-- Finish downloading benchmark")
endif()

add_library(BenchmarkUtility INTERFACE)
target_link_libraries(BenchmarkUtility INTERFACE benchmark::benchmark)
