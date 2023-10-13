#include "PDCollisionHandler.hpp"

template<>
Caster<Collider>* Caster<Collider>::_the_factory = nullptr;

void PDCollisionHandler::BindObjects(
	const typename std::vector<Object>::const_iterator &begin,
	const typename std::vector<Object>::const_iterator &end
) {
	_obj_container.BindObjects(begin, end);
	int total_dof = 0;
	for (const auto& obj : _obj_container._objs) {
		_offsets.push_back(total_dof);
		total_dof += obj.GetDOF();
	}
}

void PDCollisionHandler::BindColliders(
	const typename std::vector<Object>::const_iterator &begin,
	const typename std::vector<Object>::const_iterator &end
) {
	_collider_container.BindObjects(begin, end);
}

void PDCollisionHandler::ComputeConstraintSet(const Ref<const VectorXd>& x) {
	int obj_id = 0;
	for (const auto& obj : _obj_container._objs) {
		obj.ComputeCollisionVertex(x.segment(_offsets[obj_id], obj.GetDOF()));
		obj_id++;
	}

	_penetrated_vertices.clear();
	for (const auto& collider : _collider_container._objs) {
		obj_id = 0;
		for (const auto& obj : _obj_container._objs) {
			const auto& vertices = obj.GetCollisionVertices();
			const int num_vertices = vertices.rows();
			for (int i = 0; i < num_vertices; i++) {
				const Vector3d vertex = vertices.row(i).transpose();
				if (collider.Intersect(vertex)) {
					const Vector3d projection = collider.SurfaceProject(vertex);
					const Vector3d normal = (projection - vertex).normalized();
					_penetrated_vertices.emplace_back(PenetratedVertex{
						obj_id, i, normal, normal.dot(projection)
					});
				}
			}
			obj_id++;
		}
	}
}

void PDCollisionHandler::GetGlobalMatrix(COO &coo, int x_offset, int y_offset) const {
	for (const auto& penetrated_vertex : _penetrated_vertices){
		const int& obj_id = penetrated_vertex._obj_id;
		const int& vertex_id = penetrated_vertex._vertex_id;
		const auto& obj = _obj_container._objs[obj_id];
		obj.GetCollisionVertexDerivative(vertex_id)
		.RightTransposeProduct(obj.GetCollisionVertexDerivative(vertex_id))
		.ToSparse(
			_collision_stiffness / 2,
			coo,
			x_offset + _offsets[obj_id],
			y_offset + _offsets[obj_id]
		);
	}
}

void PDCollisionHandler::LocalProject(const Ref<const VectorXd> &x, Ref<VectorXd> y, int offset) const {
	int total_dof = 0;
	for (const auto& obj : _obj_container._objs) {
		obj.ComputeCollisionVertex(x.segment(total_dof, obj.GetDOF()));
		total_dof += obj.GetDOF();
	}

	for (const auto& penetrated_vertex : _penetrated_vertices){
		const int& obj_id = penetrated_vertex._obj_id;
		const int& vertex_id = penetrated_vertex._vertex_id;
		const Vector3d& normal = penetrated_vertex._normal;
		const double& offset = penetrated_vertex._offset;
		const auto& obj = _obj_container._objs[obj_id];

		const Vector3d vertex = obj.GetCollisionVertices().row(vertex_id).transpose();
		const Vector3d target =
			normal.dot(vertex) < offset ? 
			vertex - vertex.dot(normal) * normal + normal * offset :
			vertex;
		obj.GetCollisionVertexDerivative(vertex_id).RightProduct(target * _collision_stiffness / 2, y.segment(_offsets[obj_id], obj.GetDOF()));
	}
}
