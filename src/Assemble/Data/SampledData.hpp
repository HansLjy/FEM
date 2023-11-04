#pragma once
#include "Data.hpp"

struct SampledObjectData : public BasicData {
	SampledObjectData(const SampledObjectData& rhs) = delete;
	SampledObjectData(SampledObjectData&& rhs) = default;

	// <- the codimension argument is actually a misnomer, it is the dimension of the object
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

// Sometimes it is an agony to set the DOF to 0.
// An easy workaround would be to set the mass to a very large value
struct FixedSampledObjectData : public SampledObjectData {
	static FixedSampledObjectData CreateFromConfig(const json& config);
	static FixedSampledObjectData GetFixedSampledObjectData(const VectorXd& x, const MatrixXi& topo, int codimension, double single_mass);

protected:
	using SampledObjectData::SampledObjectData;
	FixedSampledObjectData(SampledObjectData&& rhs) : SampledObjectData(std::move(rhs)) {}
};