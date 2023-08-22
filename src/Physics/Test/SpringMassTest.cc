#include "FiniteDifference.h"
#include "Model/MassSpringModel.hpp"
#include "gtest/gtest.h"

struct TestSpringMassData {
	// Generate complete connection on a circle
	TestSpringMassData(int num_points, double stiffness)
	: _num_edges(num_points * (num_points - 1) / 2), _stiffness(stiffness) {
		_edge_topo.resize(_num_edges, 2);
		int cnt_edges = 0;
		for (int i = 0; i < num_points; i++) {
			for (int j = i + 1; j < num_points; j++) {
				_edge_topo.row(cnt_edges++) << i, j;
			}
		}
	}

	int _num_edges;
	double _stiffness;
	MatrixXi _edge_topo;
	VectorXd _rest_length;
};

TEST(SpringMassTest, GradientTest) {

}

