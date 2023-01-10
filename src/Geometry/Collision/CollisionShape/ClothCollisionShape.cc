#include "ClothCollisionShape.h"
#include "Object/Cloth.h"
#include "GeometryUtil.h"

void ClothCollisionShape::Bind(const Object &obj) {
	const auto& cloth = dynamic_cast<const Cloth&>(obj);
	_face_topo = cloth._topo;
	_edge_topo = GetEdgeTopo(cloth._topo);
	_vertex_projections.resize(cloth.GetDOF(), cloth.GetDOF());
	_vertex_projections.setIdentity();
}

void ClothCollisionShape::ComputeCollisionShape(const Ref<const VectorXd> &x) {
	_vertices = StackVector<double, 3>(x);
}

Vector3d ClothCollisionShape::GetCollisionVertexVelocity(const Ref<const VectorXd> &x, int idx) const {
	return x.segment<3>(3 * idx);
}
