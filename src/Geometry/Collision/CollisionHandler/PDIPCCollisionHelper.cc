#include "PDIPCCollisionHelper.hpp"
#include "Collision/CollisionUtil/ProcessPrimitivePair.hpp"
#include "GeometryUtil.hpp"
#include "Collision/CollisionUtility.h"

template<>
Caster<MassedCollisionInterface>* Caster<MassedCollisionInterface>::_the_factory = nullptr;

double GetStiffness(double kappa, double d_hat, double distance) {
	return kappa * std::log(distance / d_hat);
}

void PDIPCCollisionHandler::ClearConstraintSet() {
	_projection_infos.clear();
}

void PDIPCCollisionHandler::AddCollisionPairs(
	const std::vector<MassedCollisionInterface>& objs,
	const std::vector<int>& offsets,
	const std::vector<PrimitivePair> &constraint_set,
	const std::vector<double> & local_tois,
	const double global_toi
) {
	int primitive_pair_id = 0;

	for (const auto& primitive_pair : constraint_set) {
		if (local_tois[primitive_pair_id] > 1) {
			continue;
		}
		std::cerr << "fuck: " << __FILE__ << ":" << __LINE__ << std::endl;
		std::cout << (primitive_pair._type == CollisionType::kEdgeEdge ? "Edge-Edge " : "Vertex-Face ")
				  << primitive_pair._primitive_id1 << " " << primitive_pair._primitive_id2 << std::endl;

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

				double distance_toi = GetVPDistance(
					vertex + global_toi * vertex_velocity,
					face1 + global_toi * face_velocity1,
					face2 + global_toi * face_velocity2,
					face3 + global_toi * face_velocity3
				);

				if (distance_toi > _d_hat) {
					distance_toi = 0.9 * _d_hat;
				}
				const double stiffness = GetStiffness(_kappa, _d_hat, distance_toi);
				const double m_vertex = obj1.GetCollisionVertexMass(vertex_index);
				const double m_face = obj2.GetCollisionVertexMass(face_indices[0])
									+ obj2.GetCollisionVertexMass(face_indices[1])
									+ obj2.GetCollisionVertexMass(face_indices[2]);
				const double local_toi = local_tois[primitive_pair_id];
				const Vector3d vertex_toi = vertex + vertex_velocity * local_toi;
				const Vector3d face_toi1 = face1 + face_velocity1 * local_toi;
				const Vector3d face_toi2 = face2 + face_velocity2 * local_toi;
				const Vector3d face_toi3 = face3 + face_velocity3 * local_toi;
				Vector3d barycentric_coord;
				barycentric_coord.segment<2>(1) = GeometryUtil::GetClosestPoint(vertex_toi, face_toi1, face_toi2, face_toi3);
				barycentric_coord(0) = 1 - barycentric_coord(1) - barycentric_coord(2);
				const Vector3d closest_point = barycentric_coord(0) * face_toi1
											+ barycentric_coord(1) * face_toi2
											+ barycentric_coord(2) * face_toi3;
				const Vector3d closest_point_velocity = barycentric_coord(0) * face_velocity1
													+ barycentric_coord(1) * face_velocity2
													+ barycentric_coord(2) * face_velocity3;
				const Vector3d normal = (vertex_toi - closest_point).normalized();
				double v_vertex = vertex_velocity.dot(normal);
				double v_vertex_after;
				double v_face = closest_point_velocity.dot(normal);
				double v_face_after;
				CollisionUtils::PerfectElasticCollision(v_vertex, v_face, m_vertex, m_face, v_vertex_after, v_face_after);
				const Vector3d vertex_velocity_after = vertex_velocity - (vertex_velocity.dot(normal) - v_vertex_after) * normal;
				const Vector3d face_velocity_after1 = face_velocity1 - (face_velocity1.dot(normal) - v_face_after) * normal;
				const Vector3d face_velocity_after2 = face_velocity2 - (face_velocity2.dot(normal) - v_face_after) * normal;
				const Vector3d face_velocity_after3 = face_velocity3 - (face_velocity3.dot(normal) - v_face_after) * normal;
				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id1,
					vertex_index,
					stiffness,
					vertex + local_toi * vertex_velocity + (1 - local_toi) * vertex_velocity_after
				));
				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id2,
					face_indices[0],
					stiffness,
					face1 + local_toi * face_velocity1 + (1 - local_toi) * face_velocity_after1
				));
				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id2,
					face_indices[1],
					stiffness,
					face1 + local_toi * face_velocity2 + (1 - local_toi) * face_velocity_after2
				));
				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id2,
					face_indices[2],
					stiffness,
					face1 + local_toi * face_velocity3 + (1 - local_toi) * face_velocity_after3
				));
				break;
			}
			case CollisionType::kEdgeEdge: {
				const RowVector2i edge_indices1 = obj1.GetCollisionEdgeTopo().row(primitive_pair._obj_id1);
				const Vector3d edge11 = obj1.GetCollisionVertices().row(edge_indices1[0]);
				const Vector3d edge12 = obj1.GetCollisionVertices().row(edge_indices1[1]);
				const Vector3d edge_velocity11 = obj1.GetCollisionVertexVelocity(edge_indices1[0]);
				const Vector3d edge_velocity12 = obj1.GetCollisionVertexVelocity(edge_indices1[1]);
				
				const RowVector2i edge_indices2 = obj2.GetCollisionEdgeTopo().row(primitive_pair._obj_id2);
				const Vector3d edge21 = obj2.GetCollisionVertices().row(edge_indices2[0]);
				const Vector3d edge22 = obj2.GetCollisionVertices().row(edge_indices2[1]);
				const Vector3d edge_velocity21 = obj2.GetCollisionVertexVelocity(edge_indices2[0]);
				const Vector3d edge_velocity22 = obj2.GetCollisionVertexVelocity(edge_indices2[1]);
				
				double distance_toi = GetLLDistance(
					edge11 + global_toi * edge_velocity11,
					edge12 + global_toi * edge_velocity12,
					edge21 + global_toi * edge_velocity21,
					edge22 + global_toi * edge_velocity22
				);
				if (distance_toi > _d_hat) {
					distance_toi = 0.9 * _d_hat;
				}

				const double stiffness = GetStiffness(_kappa, _d_hat, distance_toi);
				
				const double m_edge1 = obj1.GetCollisionVertexMass(edge_indices1[0])
									+ obj1.GetCollisionVertexMass(edge_indices1[1]);
				const double m_edge2 = obj2.GetCollisionVertexMass(edge_indices2[0])
									+ obj2.GetCollisionVertexMass(edge_indices2[1]);
				const double local_toi = local_tois[primitive_pair_id];
				const Vector3d edge_toi11 = edge11 + edge_velocity11 * local_toi;
				const Vector3d edge_toi12 = edge12 + edge_velocity12 * local_toi;
				const Vector3d edge_toi21 = edge21 + edge_velocity21 * local_toi;
				const Vector3d edge_toi22 = edge22 + edge_velocity22 * local_toi;

				Vector2d barycentric_coord;
				barycentric_coord = GeometryUtil::GetLineLineClosestPoint(
					edge_toi11, edge_toi12,
					edge_toi21, edge_toi22
				);
				const Vector3d closest_point1 = barycentric_coord(0) * edge_toi12
											+ (1 - barycentric_coord(0)) * edge_toi11;
				const Vector3d closest_point2 = barycentric_coord(1) * edge_toi22
											+ (1 - barycentric_coord(1)) * edge_toi21;
				const Vector3d closest_point_velocity1 = barycentric_coord(0) * edge_velocity12
													+ (1 - barycentric_coord(0)) * edge_velocity11;
				const Vector3d closest_point_velocity2 = barycentric_coord(1) * edge_velocity22
													+ (1 - barycentric_coord(1)) * edge_velocity21;
				
				const Vector3d normal = (closest_point1 - closest_point2).normalized();
				double normal_velocity1 = closest_point_velocity1.dot(normal);
				double normal_velocity_after1;
				double normal_velocity2 = closest_point_velocity2.dot(normal);
				double normal_velocity_after2;
				CollisionUtils::PerfectElasticCollision(
					normal_velocity1, normal_velocity2,
					m_edge1, m_edge2,
					normal_velocity_after1, normal_velocity_after2
				);
				const Vector3d edge_velocity_after11 = edge_velocity11 - (edge_velocity11.dot(normal) - normal_velocity_after1) * normal;
				const Vector3d edge_velocity_after12 = edge_velocity12 - (edge_velocity12.dot(normal) - normal_velocity_after1) * normal;
				const Vector3d edge_velocity_after21 = edge_velocity21 - (edge_velocity21.dot(normal) - normal_velocity_after2) * normal;
				const Vector3d edge_velocity_after22 = edge_velocity22 - (edge_velocity22.dot(normal) - normal_velocity_after2) * normal;
				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id1, edge_indices1[0], stiffness,
					edge11 + local_toi * edge_velocity11 + (1 - local_toi) * edge_velocity_after11
				));
				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id1, edge_indices1[1], stiffness,
					edge12 + local_toi * edge_velocity12 + (1 - local_toi) * edge_velocity_after12
				));
				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id2, edge_indices2[0], stiffness,
					edge21 + local_toi * edge_velocity21 + (1 - local_toi) * edge_velocity_after21
				));
				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id2, edge_indices2[1], stiffness,
					edge22 + local_toi * edge_velocity22 + (1 - local_toi) * edge_velocity_after22
				));

				break;
			}
		}
		primitive_pair_id++;
	}
}

void PDIPCCollisionHandler::BarrierLocalProject(
	const std::vector<MassedCollisionInterface> &objs,
	const std::vector<int>& offsets,
	Ref<VectorXd> y
) const {
	for (const auto& projection_info : _projection_infos) {
		const auto& obj = objs[projection_info._obj_id];
		obj.GetCollisionVertexDerivative(projection_info._vertex_id)
		.RightProduct(
			projection_info._stiffness / 2 * projection_info._target_position,
			y.segment(offsets[projection_info._obj_id], obj.GetDOF())
		);
	}
}

void PDIPCCollisionHandler::GetBarrierGlobalMatrix(
	const std::vector<MassedCollisionInterface> &objs,
	const std::vector<int>& offsets,
	COO& coo, int x_offset, int y_offset
) const {
	for (const auto& projection_info : _projection_infos) {
		const auto& obj = objs[projection_info._obj_id];
		obj.GetCollisionVertexDerivative(projection_info._vertex_id)
		.RightTransposeProduct(
			obj.GetCollisionVertexDerivative(projection_info._vertex_id)
		).ToSparse(
			projection_info._stiffness / 2, coo,
			x_offset + offsets[projection_info._obj_id],
			y_offset + offsets[projection_info._obj_id]
		);
	}
}

void PDIPCCollisionHandler::GetBarrierGlobalMatrix(
	const std::vector<MassedCollisionInterface> &objs,
	const std::vector<int>& offsets,
	int total_dof,
	SparseMatrixXd& global_matrix
) const {
	COO coo;
	GetBarrierGlobalMatrix(objs, offsets, coo, 0, 0);
	global_matrix.resize(total_dof, total_dof);
	global_matrix.setFromTriplets(coo.begin(), coo.end());
}
