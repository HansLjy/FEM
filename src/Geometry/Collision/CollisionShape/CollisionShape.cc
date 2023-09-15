#include "CollisionShape.h"
#include "unsupported/Eigen/KroneckerProduct"

// void AffineDecomposedCollisionShape::Bind(const Object &obj) {
// 	_obj = const_cast<AffineDecomposedObject*>(dynamic_cast<const AffineDecomposedObject*>(&obj));	// sorry about that...
// 	_num_points = _obj->_proxy->_collision_shape->GetCollisionVerticeNumber();
// 	int num_edges = _obj->_proxy->_collision_shape->GetCollisionEdgeTopo().rows();
// 	int num_faces = _obj->_proxy->_collision_shape->GetCollisionFaceTopo().rows();
// 	for (const auto& child : _obj->_children) {
// 		_num_points += child->_collision_shape->GetCollisionVerticeNumber();
// 		num_edges += child->_collision_shape->GetCollisionEdgeTopo().rows();
// 		num_faces += child->_collision_shape->GetCollisionFaceTopo().rows();
// 	}
// 	_edge_topo.resize(num_edges, 2);
// 	_face_topo.resize(num_faces, 3);
// 	num_edges = _obj->_proxy->_collision_shape->GetCollisionEdgeTopo().rows();
// 	num_faces = _obj->_proxy->_collision_shape->GetCollisionFaceTopo().rows();
// 	_edge_topo.topRows(num_edges) = _obj->_proxy->_collision_shape->GetCollisionEdgeTopo();
// 	_face_topo.topRows(num_faces) = _obj->_proxy->_collision_shape->GetCollisionFaceTopo();
// 	int points_offset = _obj->_proxy->_collision_shape->GetCollisionVerticeNumber();
// 	_children_vertices_offset.push_back(points_offset);
// 	for (const auto& child : _obj->_children) {
// 		const int cur_num_edges = child->_collision_shape->GetCollisionEdgeTopo().rows();
// 		const int cur_num_faces = child->_collision_shape->GetCollisionFaceTopo().rows();
// 		_edge_topo.middleRows(num_edges, cur_num_edges) = child->_collision_shape->GetCollisionEdgeTopo().array() + points_offset;
// 		_face_topo.middleRows(num_faces, cur_num_faces) = child->_collision_shape->GetCollisionFaceTopo().array() + points_offset;
// 		num_edges += cur_num_edges;
// 		num_faces += cur_num_faces;
// 		points_offset += child->_collision_shape->GetCollisionVerticeNumber();
// 		_children_vertices_offset.push_back(points_offset);
// 	}
// 	_children_vertex_derivative.clear();
// 	int current_child_offset = 0, current_child_id = 0;
// 	int current_vertex_id = 0;
// 	int proxy_dof = _obj->_proxy->GetDOF();
// 	int total_dof = proxy_dof + _obj->_children.size() * 9;
// 	for (const auto& child : _obj->_children) {
// 		MatrixXd submatrix(proxy_dof + 9, 3);
// 		submatrix.topRows(proxy_dof) = _obj->_children_projections[current_child_id].transpose();
// 		submatrix.bottomRows<9>().setZero();	// wait until UpdateDerivative was called
// 		for (int i = 0; i < child->_collision_shape->GetCollisionVerticeNumber(); i++, current_vertex_id++) {
// 			_children_vertex_derivative.push_back(BlockVector(
// 				total_dof, 2,
// 				{0, proxy_dof + current_child_offset},
// 				{proxy_dof, 9},
// 				submatrix
// 			));
// 		}
// 		current_child_id++;
// 		current_child_offset += 9;
// 	}
// };

// void AffineDecomposedCollisionShape::ComputeCollisionShape(const Ref<const VectorXd> &x) {
// 	_obj->_proxy->_collision_shape->ComputeCollisionShape(x.head(_obj->_proxy->GetDOF()));
// 	_obj->_total_vertices.topRows(_obj->_proxy->_collision_shape->GetCollisionVerticeNumber())
// 		= _obj->_proxy->_collision_shape->GetCollisionVertices();
// }

// const MatrixXd& AffineDecomposedCollisionShape::GetCollisionVertices() const {
// 	return _obj->_total_vertices;
// }


// const BlockVector & AffineDecomposedCollisionShape::GetCollisionVertexDerivative(int idx) const {
// 	const int proxy_vertex_number = _obj->_proxy->_collision_shape->GetCollisionVerticeNumber();
// 	return idx < proxy_vertex_number
// 			? _obj->_proxy->_collision_shape->GetCollisionVertexDerivative(idx)
// 			: _children_vertex_derivative[idx - proxy_vertex_number];
// }

// void AffineDecomposedCollisionShape::UpdateDerivative() {
// 	const int proxy_vertex_number = _obj->_proxy->_collision_shape->GetCollisionVerticeNumber();
// 	int current_vertices_id = proxy_vertex_number;
// 	for (const auto& child : _obj->_children) {
// 		for (int i = 0; i < child->_collision_shape->GetCollisionVerticeNumber(); i++) {
// 			_children_vertex_derivative[current_vertices_id - proxy_vertex_number]._submatrix.bottomRows<9>() = Eigen::kroneckerProduct(_obj->_total_vertices.row(current_vertices_id).transpose(), Matrix3d::Identity());
// 		}
// 	}
// }

// Vector3d AffineDecomposedCollisionShape::GetCollisionVertexVelocity(const Ref<const VectorXd> &v, int idx) const {
// 	const int proxy_vertex_number = _obj->_proxy->_collision_shape->GetCollisionVerticeNumber();
// 	if (idx < proxy_vertex_number) {
// 		return _obj->_proxy->_collision_shape->GetCollisionVertexVelocity(v.head(_obj->_proxy->GetDOF()), idx);
// 	} else {
// 		int child_id = std::lower_bound(_children_vertices_offset.begin(), _children_vertices_offset.end(), idx) - _children_vertices_offset.begin();
// 		if (_children_vertices_offset[child_id] > idx) {
// 			child_id--;
// 		}
// 		int proxy_dof = _obj->_proxy->GetDOF();
// 		Vector3d velocity = _obj->_children_projections[child_id] * v.head(proxy_dof);
// 		Vector3d x = _obj->_total_vertices.row(idx);
// 		for (int i = 0, offset = proxy_dof + 9 * child_id; i < 3; i++, offset += 3) {
// 			velocity += x(i) * v.segment<3>(offset);
// 		}
// 		return velocity;
// 	}
// }