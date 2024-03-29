#pragma once
#include "EigenAll.h"
#include "TopoUtil.hpp"
#include "Collision/CollisionShape/CollisionShape.hpp"

struct BasicData : public CollisionData {
	BasicData(const BasicData& rhs) = delete;
	BasicData(BasicData&& rhs) = default;

	int _dof;
	VectorXd _x, _v;

protected:
	BasicData(const VectorXd& x, const VectorXd& v) : _dof(x.size()), _x(x), _v(v) {}
	BasicData(const VectorXd& x) : BasicData(x, VectorXd::Zero(x.size())) {}
};

template<class ProxyData>
struct ReducedObjectData : public BasicData {
	ReducedObjectData<ProxyData>(const VectorXd& x, ProxyData* proxy, const SparseMatrixXd&& base, const VectorXd&& shift) : BasicData(x), _proxy(proxy), _base(base), _shift(shift) {}

	virtual ~ReducedObjectData() {delete _proxy;}
	
	ProxyData* _proxy;
	SparseMatrixXd _base;
	VectorXd _shift;
};

struct FixedObjectData : public BasicData {
	FixedObjectData() : BasicData(VectorXd(0)) {}
};
