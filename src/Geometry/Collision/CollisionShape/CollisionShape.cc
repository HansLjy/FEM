#include "CollisionShape.h"
#include "FixedCollisionShape/RectangleCollisionShape.h"

BEGIN_DEFINE_XXX_FACTORY(FixedCollisionShape)
    ADD_PRODUCT("rectangle", RectangleCollisionShape)
END_DEFINE_XXX_FACTORY

void SampledCollisionShape::Bind(const Object &obj) {
	const auto sampled_obj = dynamic_cast<const SampledObject*>(&obj);
	_edge_topo = sampled_obj->_edge_topo;
	_face_topo = sampled_obj->_face_topo;

	const int dof = obj.GetDOF();
	for (int i = 0, i3 = 0; i < sampled_obj->_num_points; i++, i3 += 3) {
		_vertex_derivatives.push_back(BlockVector(dof, 1, {i3}, {3}, Matrix3d::Identity()));
	}
}

void SampledCollisionShape::ComputeCollisionShape(const Ref<const VectorXd> &x) {
	_vertices = StackVector<double, 3>(x);
}

Vector3d SampledCollisionShape::GetCollisionVertexVelocity(const Ref<const VectorXd> &v, int idx) const {
	return v.segment<3>(3 * idx);
}

const BlockVector& SampledCollisionShape::GetVertexDerivative(int idx) const {
	return _vertex_derivatives[idx];
}