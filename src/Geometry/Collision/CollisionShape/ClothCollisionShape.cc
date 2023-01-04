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
	const int rows = x.size() / 3;
	// _vertices = Eigen::Map<Matrix<int, Dynamic, 3>, 0, Eigen::Stride<Dynamic, Dynamic>> (
	// 	(int*)x.data(), rows, 3,
	// 	Eigen::Stride<Dynamic, Dynamic>(
	// 		x.outerStride(), x.innerStride()
	// 	)
	// );
}