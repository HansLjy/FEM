#pragma once
#include "Data.hpp"

struct SampledObjectData : public BasicData {
	SampledObjectData(const SampledObjectData& rhs) = delete;
	SampledObjectData(SampledObjectData&& rhs) = default;

	int _num_points;
	int _num_edges;
	int _num_faces;
	MatrixXi _edge_topo;
	MatrixXi _face_topo;
	MatrixXi _tet_topo;

	double _total_mass;
	VectorXd _mass;

protected:
	SampledObjectData(const VectorXd& x, const VectorXd& mass, int dimension, const MatrixXi& topo);
};