#include "CurveCollisionShape.h"
#include "Object/Curve.h"

void CurveCollisionShape::Bind(const Object &obj) {
	const auto& curve = dynamic_cast<const Curve&>(obj);
	
	_edge_topo.resize(curve._num_points - 1, 2);
	for (int i = 0; i < curve._num_points - 1; i++) {
		_edge_topo.row(i) << i, i + 1;
	}
	_vertex_projections.resize(curve._dof, curve._dof);
	_vertex_projections.setIdentity();
}

void CurveCollisionShape::ComputeCollisionShape(const Ref<const VectorXd> &x) {
	_vertices = x;
}

Vector3d CurveCollisionShape::GetCollisionVertexVelocity(const Ref<const VectorXd> &v, int idx) const {
	return v.segment<3>(3 * idx);
}
