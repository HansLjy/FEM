#pragma once

#include "Data.hpp"

using Eigen::Matrix3Xd;

struct SpringMassData : public SampledObjectData {
private:
	struct EdgeTopo {
		EdgeTopo(int from, int to) : _from(from), _to(to) {}
		int _from, _to;
	};
	

public:
	SpringMassData(const VectorXd& x_rest, const MatrixXi& topo, double density, double stiffness, int initial_vertices_number);

	void AddTriangle(int id1, int id2, const Vector3d& position);
	int _num_edges;
	int _num_faces;

	double _stiffness;
	double _density;	// surface density
	VectorXd _x_rest;
};