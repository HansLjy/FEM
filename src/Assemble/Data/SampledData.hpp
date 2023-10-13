#pragma once
#include "Data.hpp"

struct SampledObjectData : public BasicData {
	SampledObjectData(const SampledObjectData& rhs) = delete;
	SampledObjectData(SampledObjectData&& rhs) = default;

	static SampledObjectData GetSampledObjectData(const VectorXd& x, const VectorXd& mass, const MatrixXi& topo, int codimension);

	int _num_points;
	int _num_edges;
	int _num_faces;
	MatrixXi _edge_topo;
	MatrixXi _face_topo;
	MatrixXi _tet_topo;

	double _total_mass;
	VectorXd _mass;

protected:
	SampledObjectData(
		const VectorXd& x,
		const VectorXd& mass,
		const MatrixXi& tet_topo,
		const MatrixXi& face_topo,
		const MatrixXi& edge_topo
	);
};