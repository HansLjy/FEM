#pragma once
#include "Data.hpp"

VectorXd GenerateMass2D(const VectorXd& x, double density, const MatrixXi& topo);
VectorXd GenerateMass3D(const VectorXd& x, double density, const MatrixXi& topo);

struct SampledObjectData : public BasicData {
	SampledObjectData(const std::string& filename, double density, int dimension);
	SampledObjectData(const VectorXd& x, double density, int dimension, const MatrixXi& topo);
	SampledObjectData(const VectorXd& x, const VectorXd& mass, int dimension, const MatrixXi& topo);

	int _num_points;
	int _num_edges;
	int _num_faces;
	MatrixXi _edge_topo;
	MatrixXi _face_topo;
	MatrixXi _tet_topo;

	double _total_mass;
	VectorXd _mass;
};

// TODO: Currently only support 2D objects, need 3D support
struct DynamicSampledObjectData : public SampledObjectData {
	DynamicSampledObjectData(const std::string& filename, double density, int IFN);
	DynamicSampledObjectData(const VectorXd& x, double density, const MatrixXi& topo, int IFN);

	void AddFace(int id1, int id2, int id3);
	void AddFace(int id1, int id2, const Vector3d& position);

	double _density;
	VectorXd _x_rest;

protected:
	void SetIFN(int IFN);
};