#include "ReducedTreeTrunkCollisionShape.h"
#include "Object/ReducedTreeTrunk.h"

void ReducedTreeTrunkCollisionShape::Bind(const Object &obj) {
	_reduced_tree_trunk = dynamic_cast<const ReducedTreeTrunk*>(&obj);
	_vertex_derivatives.push_back(BlockVector(9, 0, {}, {}, MatrixXd(0, 3)));
	for (int i = 1; i <= 3; i++) {
		_vertex_derivatives.push_back(BlockVector(9, 1, {3 * (i - 1)}, {3}, Matrix3d::Identity()));
	}
	_vertices.resize(4, 3);
	_num_points = 4;
	_edge_topo.resize(6, 2);
	_edge_topo <<
		0, 1,
		1, 2,
		2, 3,
		0, 2,
		1, 3,
		0, 3;
	_face_topo.resize(4, 3);
	_face_topo <<
		0, 1, 2,
		0, 3, 1,
		0, 2, 3,
		1, 3, 2;
}

void ReducedTreeTrunkCollisionShape::ComputeCollisionShape(const Ref<const VectorXd> &x) {
	_vertices.row(0) = _reduced_tree_trunk->_fixed_point.transpose();
	for (int i = 1, i3 = 0; i <= 3; i++, i3 += 3) {
		_vertices.row(i) = x.segment<3>(i3).transpose();
	}
}

Vector3d ReducedTreeTrunkCollisionShape::GetCollisionVertexVelocity(const Ref<const VectorXd> &v, int idx) const {
	return idx == 0 ? Vector3d::Zero() : Vector3d(v.segment<3>(3 * (idx - 1)));
}

const BlockVector& ReducedTreeTrunkCollisionShape::GetVertexDerivative(int idx) const {
	return _vertex_derivatives[idx];
}
