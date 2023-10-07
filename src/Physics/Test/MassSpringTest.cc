#include <fstream>
#include "DerivativeTest.h"
#include "EnergyModel/MassSpringEnergyModel.hpp"
#include "gtest/gtest.h"

struct Grid {
	Grid(int num_points, int num_edges, double stiffness, const MatrixXi& edge_topo, const VectorXd& rest_length) : _dof(num_points * 3), _num_edges(num_edges), _stiffness(stiffness), _edge_topo(edge_topo), _rest_length(rest_length) {}

	int _dof;
	int _num_edges;
	double _stiffness;
	MatrixXi _edge_topo;
	VectorXd _rest_length;
};


Grid* GenerateTestSrpingMassData(int num_points, int num_edges, double stiffness) {
	MatrixXi edge_topo(num_edges, 2);
	VectorXd rest_length(num_edges);
	for (int i = 0; i < num_edges; i++) {
		const int id1 = std::rand() % num_points;
		const int id2 = (id1 + std::rand() % (num_points - 1) + 1) % num_points;
		edge_topo.row(i) << id1, id2;
		rest_length(i) = 1.0 * std::rand() / RAND_MAX;
	}
	return new Grid(num_points, num_edges, stiffness, edge_topo, rest_length);
}

const double gradient_step = 1e-8;
const double hessian_step = 1e-4;

TEST(MassSpringTest, GradientTest) {
	std::fstream config_file(PHYSICS_TEST_CONFIG_PATH "/SpringMassTest.json");
	json config;
	config_file >> config;
	const int num_points = config["num-points"];
	const int num_edges = config["num-edges"];
	const double stiffness = 1.0 * std::rand() / RAND_MAX;
	const auto data = GenerateTestSrpingMassData(num_points, num_edges, stiffness);
	MassSpringEnergyModel physics;
	
	GenerateDerivative(data, physics, gradient_step, hessian_step)
	
	// PrintGradient();
	// PrintHessian();

	EXPECT_NEAR((numeric_gradient - analytic_gradient).norm() / numeric_gradient.size(), 0, 1e-6);
	EXPECT_NEAR((numeric_hessian - analytic_hessian).norm() / numeric_hessian.size(), 0, 1e-5);

}

