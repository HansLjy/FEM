#pragma once

#include "CollisionShape.h"

class ReducedTreeTrunk;

class ReducedTreeTrunkCollisionShape : public CollisionShape {
public:
	template<class Object> void Precompute(const Object* obj);
	template<class Object> void ComputeCollisionShape(const Object* obj, const Ref<const VectorXd>& x);
	template<class Object> const BlockVector& GetVertexDerivative(const Object* obj, int idx) const;
	template<class Object> double GetMaxVelocity(const Object* obj, const Ref<const VectorXd> &v) const;
	template<class Object> Vector3d GetCollisionVertexVelocity(const Object* obj, const Ref<const VectorXd>& v, int idx) const;

protected:
	std::vector<BlockVector> _vertex_derivatives;
};

template<class Object>
void ReducedTreeTrunkCollisionShape::Precompute(const Object* obj) {
	_vertex_derivatives.push_back(BlockVector(9, 0, {}, {}, MatrixXd(0, 3)));
	for (int i = 1; i <= 3; i++) {
		_vertex_derivatives.push_back(BlockVector(9, 1, {3 * (i - 1)}, {3}, Matrix3d::Identity()));
	}
	_collision_vertices.resize(4, 3);
	_collision_edge_topo.resize(6, 2);
	_collision_edge_topo <<
		0, 1,
		1, 2,
		2, 3,
		0, 2,
		1, 3,
		0, 3;
	_collision_face_topo.resize(4, 3);
	_collision_face_topo <<
		0, 1, 2,
		0, 3, 1,
		0, 2, 3,
		1, 3, 2;
}

template<class Object>
void ReducedTreeTrunkCollisionShape::ComputeCollisionShape(const Object* obj, const Ref<const VectorXd> &x) {
	_collision_vertices.row(0) = obj->_fixed_point.transpose();
	for (int i = 1, i3 = 0; i <= 3; i++, i3 += 3) {
		_collision_vertices.row(i) = x.segment<3>(i3).transpose();
	}
}

template<class Object>
const BlockVector& ReducedTreeTrunkCollisionShape::GetVertexDerivative(const Object* obj, int idx) const {
	return _vertex_derivatives[idx];
}

template<class Object>
double ReducedTreeTrunkCollisionShape::GetMaxVelocity(const Object* obj, const Ref<const VectorXd> &v) const {
	double max_velocity = 0;
	for (int i = 0; i < 3; i++) {
		max_velocity = std::max(max_velocity, v.segment<3>(i * 3).norm());
	}
	return max_velocity;
}

template<class Object>
Vector3d ReducedTreeTrunkCollisionShape::GetCollisionVertexVelocity(const Object *obj, const Ref<const VectorXd> &v, int idx) const {
	return idx == 0 ? Vector3d::Zero() : Vector3d(v.segment<3>(3 * (idx - 1)));
}
