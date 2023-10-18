#include "TOIEstimator.hpp"
#include "Collision/CollisionUtil/ProcessPrimitivePair.hpp"

double TOIEstimator::GetTOI(
	const std::vector<PrimitivePair> &constraint_set,
	const std::vector<CollisionInterface> &objs
) const {
	double toi = 1;

	PROCESS_PRIMITIVE_PAIR(
		constraint_set, objs,
		
		double local_toi = _ccd->VertexFaceCollision(vertex, face1, face2, face3, vertex_velocity, face_velocity1, face_velocity2, face_velocity3);
		if (local_toi < toi) {
			toi = local_toi;
		},

		double local_toi = _ccd->EdgeEdgeCollision(edge11, edge12, edge21, edge22, edge_velocity11, edge_velocity12, edge_velocity21, edge_velocity22);

		if (local_toi < toi) {
			toi = local_toi;
		}
		,
	)

	return toi;
}

double TOIEstimator::GetLocalTOIs(
	const std::vector<PrimitivePair> &constraint_set,
	const std::vector<CollisionInterface> &objs,
	std::vector<double> &local_tois
) const {
	local_tois.reserve(local_tois.size() + constraint_set.size());
	double toi = 1;

	for (const auto& primitive_pair : constraint_set) {
		const auto& obj1 = objs[primitive_pair._obj_id1];
		const auto& obj2 = objs[primitive_pair._obj_id2];
		
		switch (primitive_pair._type) {
			case CollisionType::kVertexFace: {
				const int vertex_index = primitive_pair._primitive_id1;
				const Vector3d vertex = obj1.GetCollisionVertices().row(vertex_index);
				const Vector3d vertex_velocity = obj1.GetCollisionVertexVelocity(primitive_pair._primitive_id1);
				
				const RowVector3i face_indices = obj2.GetCollisionFaceTopo().row(primitive_pair._primitive_id2);
				const Vector3d face1 = obj2.GetCollisionVertices().row(face_indices[0]);
				const Vector3d face2 = obj2.GetCollisionVertices().row(face_indices[1]);
				const Vector3d face3 = obj2.GetCollisionVertices().row(face_indices[2]);
				const Vector3d face_velocity1 = obj2.GetCollisionVertexVelocity(face_indices[0]);
				const Vector3d face_velocity2 = obj2.GetCollisionVertexVelocity(face_indices[1]);
				const Vector3d face_velocity3 = obj2.GetCollisionVertexVelocity(face_indices[2]);
				
				double local_toi = _ccd->VertexFaceCollision(vertex, face1, face2, face3, vertex_velocity, face_velocity1, face_velocity2, face_velocity3);
				local_tois.emplace_back(local_toi);
				if (local_toi < toi) {
					toi = local_toi;
				}
				
				break;
			}
			case CollisionType::kEdgeEdge: {
				const RowVector2i edge_indices1 = obj1.GetCollisionEdgeTopo().row(primitive_pair._primitive_id1);
				const Vector3d edge11 = obj1.GetCollisionVertices().row(edge_indices1[0]);
				const Vector3d edge12 = obj1.GetCollisionVertices().row(edge_indices1[1]);
				const Vector3d edge_velocity11 = obj1.GetCollisionVertexVelocity(edge_indices1[0]);
				const Vector3d edge_velocity12 = obj1.GetCollisionVertexVelocity(edge_indices1[1]);
				
				const RowVector2i edge_indices2 = obj2.GetCollisionEdgeTopo().row(primitive_pair._primitive_id2);
				const Vector3d edge21 = obj2.GetCollisionVertices().row(edge_indices2[0]);
				const Vector3d edge22 = obj2.GetCollisionVertices().row(edge_indices2[1]);
				const Vector3d edge_velocity21 = obj2.GetCollisionVertexVelocity(edge_indices2[0]);
				const Vector3d edge_velocity22 = obj2.GetCollisionVertexVelocity(edge_indices2[1]);

				double local_toi = _ccd->EdgeEdgeCollision(edge11, edge12, edge21, edge22, edge_velocity11, edge_velocity12, edge_velocity21, edge_velocity22);
				local_tois.emplace_back(local_toi);
				if (local_toi < toi) {
					toi = local_toi;
				}
				break;
			}
		}
	}

	return toi;
}