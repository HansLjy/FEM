#include "PDEnergyModel/PDClothEnergyModel.hpp"
#include "benchmark/benchmark.h"

void GenerateSquareCloth(int num_points_on_edge, VectorXd& x, MatrixXi& edge_topo, std::vector<double>& rest_length) {
	const int num_points = num_points_on_edge * num_points_on_edge;
	const int num_edges = (num_points_on_edge - 1) * num_points_on_edge * 2 + (num_points_on_edge - 1) * (num_points_on_edge - 1);
	x = VectorXd::Random(num_points * 3);
	edge_topo.resize(num_edges, 2);
	int edge_cnt = 0;
	for (int i = 0; i < num_points_on_edge - 1; i++) {
		for (int j = 0; j < num_points_on_edge - 1; j++) {
			edge_topo.row(edge_cnt++) << i * num_points_on_edge + j, i * num_points_on_edge + j + 1;
			edge_topo.row(edge_cnt++) << i * num_points_on_edge + j, (i + 1) * num_points_on_edge + j;
			edge_topo.row(edge_cnt++) << i * num_points_on_edge + j, (i + 1) * num_points_on_edge + j + 1;
		}
	}
	for (int i = 0; i < num_points_on_edge - 1; i++) {
		edge_topo.row(edge_cnt++) << (num_points_on_edge - 1) * num_points_on_edge + i, (num_points_on_edge - 1) * num_points_on_edge + i + 1;
		edge_topo.row(edge_cnt++) << i * num_points_on_edge + num_points_on_edge - 1, (i + 1) * num_points_on_edge + num_points_on_edge - 1;
	}
	rest_length.clear();
	rest_length.reserve(num_edges);
	for (int i = 0; i < num_edges; i++) {
		rest_length.push_back(1);
	}
}

static void SpringEnergyBenchmark(benchmark::State& state) {
	VectorXd x;
	MatrixXi edge_topo;
	std::vector<double> rest_length;

	GenerateSquareCloth(100, x, edge_topo, rest_length);

	VectorXd y(x.size());
	for (auto _ : state) {
		PDEnergyModelFunction::GetPDSpringEnergyRHSVector<ParallelType::kNone>(x, edge_topo, rest_length, 1, y);
	}
}

BENCHMARK(SpringEnergyBenchmark);

static void SpringEnergyParallelBenchmark(benchmark::State& state) {
	VectorXd x;
	MatrixXi edge_topo;
	std::vector<double> rest_length;

	GenerateSquareCloth(100, x, edge_topo, rest_length);

	VectorXd y(x.size());
	for (auto _ : state) {
		PDEnergyModelFunction::GetPDSpringEnergyRHSVector<ParallelType::kCPU>(x, edge_topo, rest_length, 1, y);
	}
}

BENCHMARK(SpringEnergyParallelBenchmark);

BENCHMARK_MAIN();