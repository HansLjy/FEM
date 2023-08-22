#pragma once
#include "EigenAll.h"
#include "GeometryUtil.h"

struct BasicData {
	BasicData(const VectorXd& x, const VectorXd& v) : _dof(x.size()), _x(x), _v(v) {}
	BasicData(const VectorXd& x) : BasicData(x, VectorXd::Zero(x.size())) {}

	int _dof;
	VectorXd _x, _v;
};

struct SampledObjectData : public BasicData {
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

template<class ProxyData>
struct ReducedObjectData : public BasicData {
	ReducedObjectData<ProxyData>(const VectorXd& x, ProxyData* proxy, const SparseMatrixXd&& base, const VectorXd&& shift) : BasicData(x), _proxy(proxy), _base(base), _shift(shift) {}
	
	ProxyData* _proxy;
	SparseMatrixXd _base;
	VectorXd _shift;
};

struct FixedObjectData : public BasicData {
	FixedObjectData() : BasicData(VectorXd(0)) {}
};
