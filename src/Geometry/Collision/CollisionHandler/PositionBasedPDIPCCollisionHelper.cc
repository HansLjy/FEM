#include "PositionBasedPDIPCCollisionHelper.hpp"
#include "Collision/CollisionUtil/CCD/CubicSolver/CemCubicSolver.hpp"
#include "Collision/CollisionUtil/CCD/SimpleCCD.h"
#include "Collision/CollisionUtil/ProcessPrimitivePair.hpp"
#include "Collision/GeometryComputation.h"
#include "FileIO.hpp"
#include "GeometryUtil.hpp"
#include "Collision/IpcTookit/UnclassifiedDistance.hpp"
#include "Collision/CollisionUtility.h"

template<>
Caster<MassedCollisionInterface>* Caster<MassedCollisionInterface>::_the_factory = nullptr;

void PositionBasedPDIPCCollisionUtility::GetVertexFaceRebounce(
	const Vector3d &vertex,
	const Vector3d &face1, const Vector3d &face2, const Vector3d &face3,
	const Vector3d &vertex_velocity,
	const Vector3d &face_velocity1, const Vector3d &face_velocity2, const Vector3d &face_velocity3,
	const double vertex_mass,
	const double face_mass1, const double face_mass2, const double face_mass3,
	const double local_toi,
	const double d_hat,
	const double velocity_damping,
	Vector3d &vertex_after,
	Vector3d &face_after1, Vector3d &face_after2, Vector3d &face_after3
) {
	const Vector3d vertex_toi = vertex + vertex_velocity * local_toi;
	const Vector3d face_toi1 = face1 + face_velocity1 * local_toi;
	const Vector3d face_toi2 = face2 + face_velocity2 * local_toi;
	const Vector3d face_toi3 = face3 + face_velocity3 * local_toi;
	Vector3d barycentric_coord;
	Vector3d normal;
	double distance2 = LeiLan::VertexTriangleDistance(
		vertex_toi, face_toi1, face_toi2, face_toi3,
		barycentric_coord(0), barycentric_coord(1), barycentric_coord(2), normal
	);
	normal.normalize();

	const double m1 = vertex_mass;
	const double m2 = face_mass1 * barycentric_coord(0)
					+ face_mass2 * barycentric_coord(1)
					+ face_mass3 * barycentric_coord(2);
	const Vector3d closest_point_velocity = barycentric_coord(0) * face_velocity1
										  + barycentric_coord(1) * face_velocity2
										  + barycentric_coord(2) * face_velocity3;
	double normal_velocity1 = vertex_velocity.dot(normal);
	double normal_velocity2 = closest_point_velocity.dot(normal);

	const Vector3d tangent_velocity1 = vertex_velocity - normal_velocity1 * normal;
	const Vector3d tangent_velocity2 = closest_point_velocity - normal_velocity2 * normal;

	assert((tangent_velocity1 + normal_velocity1 * normal - vertex_velocity).norm() < 1e-10);
	assert((tangent_velocity2 + normal_velocity2 * normal - closest_point_velocity).norm() < 1e-10);

	const double rebounce_dist = d_hat;
	// double rebounce_dist = (1 - local_toi) * std::abs(normal_velocity1 - normal_velocity2);
	// if(rebounce_dist < 1e-6) {
	// 	rebounce_dist = 1e-6;
	// }
	// if (rebounce_dist > d_hat) {
	// 	rebounce_dist = d_hat;
	// }

	const double normal_velocity_after1 =   rebounce_dist * (m2 / (m1 + m2)) / (1 - local_toi);
	const double normal_velocity_after2 = - rebounce_dist * (m1 / (m1 + m2)) / (1 - local_toi);


	const Vector3d velocity_after1 = normal_velocity_after1 * normal + tangent_velocity1 * velocity_damping;
	const Vector3d velocity_after2 = normal_velocity_after2 * normal + tangent_velocity2 * velocity_damping;

	vertex_after = vertex + local_toi * vertex_velocity + (1 - local_toi) * velocity_after1;
	face_after1 = face1 + local_toi * face_velocity1 + (1 - local_toi) * velocity_after2;
	face_after2 = face2 + local_toi * face_velocity2 + (1 - local_toi) * velocity_after2;
	face_after3 = face3 + local_toi * face_velocity3 + (1 - local_toi) * velocity_after2;
}

void PositionBasedPDIPCCollisionUtility::GetEdgeEdgeRebounce(
	const Vector3d &edge11, const Vector3d &edge12,
	const Vector3d &edge21, const Vector3d &edge22,
	const Vector3d &edge_velocity11, const Vector3d &edge_velocity12,
	const Vector3d &edge_velocity21, const Vector3d &edge_velocity22,
	const double edge_mass11, const double edge_mass12,
	const double edge_mass21, const double edge_mass22,
	const double local_toi,
	const double d_hat,
	const double velocity_damping,
	Vector3d &edge_after11, Vector3d &edge_after12,
	Vector3d &edge_after21, Vector3d &edge_after22
) {
	const Vector3d edge_toi11 = edge11 + edge_velocity11 * local_toi;
	const Vector3d edge_toi12 = edge12 + edge_velocity12 * local_toi;
	const Vector3d edge_toi21 = edge21 + edge_velocity21 * local_toi;
	const Vector3d edge_toi22 = edge22 + edge_velocity22 * local_toi;

	Vector2d barycentric_coord;
	Vector3d normal;
	double distance2 = LeiLan::EdgeEdgeSqDistance(
		edge_toi11, edge_toi12, edge_toi21, edge_toi22,
		barycentric_coord(0), barycentric_coord(1), normal
	);
	normal.normalize();

	const double m1 = edge_mass11 * (1 - barycentric_coord(0))
					+ edge_mass12 * barycentric_coord(0);
	const double m2 = edge_mass21 * (1 - barycentric_coord(1))
					+ edge_mass22 * barycentric_coord(1);
	const Vector3d closest_point_velocity1 = barycentric_coord(0) * edge_velocity12
										   + (1 - barycentric_coord(0)) * edge_velocity11;
	const Vector3d closest_point_velocity2 = barycentric_coord(1) * edge_velocity22
										   + (1 - barycentric_coord(1)) * edge_velocity21;
	
	// const Vector3d normal = (closest_point1 - closest_point2).normalized();
	double normal_velocity1 = closest_point_velocity1.dot(normal);
	double normal_velocity2 = closest_point_velocity2.dot(normal);

	const Vector3d tangent_velocity1 = closest_point_velocity1 - normal_velocity1 * normal;
	const Vector3d tangent_velocity2 = closest_point_velocity2 - normal_velocity2 * normal;

	const double rebounce_dist = d_hat;
	// double rebounce_dist = (1 - local_toi) * std::abs(normal_velocity1 - normal_velocity2);
	// if(rebounce_dist < 1e-6) {
	// 	rebounce_dist = 1e-6;
	// }
	// if (rebounce_dist > d_hat) {
	// 	rebounce_dist = d_hat;
	// }

	const double normal_velocity_after1 =   rebounce_dist * (m2 / (m1 + m2)) / (1 - local_toi);
	const double normal_velocity_after2 = - rebounce_dist * (m1 / (m1 + m2)) / (1 - local_toi);

	const Vector3d edge_velocity_after1 = normal_velocity_after1 * normal + tangent_velocity1 * velocity_damping;
	const Vector3d edge_velocity_after2 = normal_velocity_after2 * normal + tangent_velocity2 * velocity_damping;

	edge_after11 = edge11 + local_toi * edge_velocity11 + (1 - local_toi) * edge_velocity_after1;
	edge_after12 = edge12 + local_toi * edge_velocity12 + (1 - local_toi) * edge_velocity_after1;
	edge_after21 = edge21 + local_toi * edge_velocity21 + (1 - local_toi) * edge_velocity_after2;
	edge_after22 = edge22 + local_toi * edge_velocity22 + (1 - local_toi) * edge_velocity_after2;
}

double PositionBasedPDIPCCollisionUtility::GetStiffness(
	double kappa, double d_hat2, double distance2, double min_stffness
) {
	if (distance2 < 1e-10) {
		// guard against zero
		distance2 = 1e-10;
	}
    if (distance2 < d_hat2) {
        return -kappa * std::log(distance2 / d_hat2) + min_stffness;
    } else {
		return min_stffness;
	}
}

void PositionBasedPDIPCCollisionHandler::ClearConstraintSet() {
	_projection_infos.clear();
	_primitive_pairs.clear();
}

void PositionBasedPDIPCCollisionHandler::AddCollisionPairs(
	const std::vector<MassedCollisionInterface>& objs,
	const std::vector<int>& offsets,
	const std::vector<PrimitivePair> &constraint_set,
	const std::vector<double> & local_tois,
	const double global_toi,
	double dt
) {
	int old_primitive_pair_id = 0;
	for (const auto& primitive_pair : _primitive_pairs) {
		const auto& obj1 = objs[primitive_pair._obj_id1];
		const auto& obj2 = objs[primitive_pair._obj_id2];
		Vector3d target_position[4];
		double new_stiffness = 0;
		double distance2 = 0;
		switch (primitive_pair._type) {
			case CollisionType::kVertexFace: {
				const int vertex_index = primitive_pair._primitive_id1;
				const Vector3d vertex = obj1.GetCollisionVertices().row(vertex_index);
				const Vector3d vertex_velocity = obj1.GetCollisionVertexVelocity(vertex_index);

				const RowVector3i face_indices = obj2.GetCollisionFaceTopo().row(primitive_pair._primitive_id2);
				const Vector3d face1 = obj2.GetCollisionVertices().row(face_indices[0]);
				const Vector3d face2 = obj2.GetCollisionVertices().row(face_indices[1]);
				const Vector3d face3 = obj2.GetCollisionVertices().row(face_indices[2]);
				const Vector3d face_velocity1 = obj2.GetCollisionVertexVelocity(face_indices[0]);
				const Vector3d face_velocity2 = obj2.GetCollisionVertexVelocity(face_indices[1]);
				const Vector3d face_velocity3 = obj2.GetCollisionVertexVelocity(face_indices[2]);

				distance2 = IPC::point_triangle_distance_unclassified(
					vertex + global_toi * vertex_velocity,
					face1 + global_toi * face_velocity1,
					face2 + global_toi * face_velocity2,
					face3 + global_toi * face_velocity3
				);
				new_stiffness = PositionBasedPDIPCCollisionUtility::GetStiffness(
					_kappa, _d_hat, 
					distance2, _kappa
				);

				if (distance2 >= _d_hat * _d_hat) {
					target_position[0] = vertex;
					target_position[1] = face1;
					target_position[2] = face2;
					target_position[3] = face3;
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

				distance2 = IPC::edge_edge_distance_unclassified(
					edge11 + global_toi * edge_velocity11,
					edge12 + global_toi * edge_velocity12,
					edge21 + global_toi * edge_velocity21,
					edge22 + global_toi * edge_velocity22
				);
				new_stiffness = PositionBasedPDIPCCollisionUtility::GetStiffness(
					_kappa, _d_hat,
					distance2, _kappa
				);

				if (distance2 >= _d_hat * _d_hat) {
					target_position[0] = edge11;
					target_position[1] = edge12;
					target_position[2] = edge21;
					target_position[3] = edge22;
				}
				break;
			}
		}

		for (int i = 0; i < 4; i++) {
			_projection_infos[old_primitive_pair_id * 4 + i]._stiffness = (
				_stiffness_blending * new_stiffness +
				(1 - _stiffness_blending) * _projection_infos[old_primitive_pair_id * 4 + i]._stiffness
			);
		}
		if (distance2 >= _d_hat * _d_hat) {
			for (int i = 0; i < 4; i++) {
				_projection_infos[old_primitive_pair_id * 4 + i]._target_position = target_position[i];
			}
		}

		old_primitive_pair_id++;
	}

	int new_primitive_pair_id = 0;
	for (const auto& primitive_pair : constraint_set) {
		double local_toi = local_tois[new_primitive_pair_id++];
		if (local_toi > 1) {
			continue;
		}
		local_toi *= 0.99;

		const auto& obj1 = objs[primitive_pair._obj_id1];
		const auto& obj2 = objs[primitive_pair._obj_id2];
		
		switch (primitive_pair._type) {
			case CollisionType::kVertexFace: {
				const int vertex_index = primitive_pair._primitive_id1;
				const Vector3d vertex = obj1.GetCollisionVertices().row(vertex_index);
				const Vector3d vertex_velocity = obj1.GetCollisionVertexVelocity(vertex_index);
				
				const RowVector3i face_indices = obj2.GetCollisionFaceTopo().row(primitive_pair._primitive_id2);
				const Vector3d face1 = obj2.GetCollisionVertices().row(face_indices[0]);
				const Vector3d face2 = obj2.GetCollisionVertices().row(face_indices[1]);
				const Vector3d face3 = obj2.GetCollisionVertices().row(face_indices[2]);
				const Vector3d face_velocity1 = obj2.GetCollisionVertexVelocity(face_indices[0]);
				const Vector3d face_velocity2 = obj2.GetCollisionVertexVelocity(face_indices[1]);
				const Vector3d face_velocity3 = obj2.GetCollisionVertexVelocity(face_indices[2]);

				_primitive_pairs.emplace_back(primitive_pair);

				Vector3d vertex_after, face_after1, face_after2, face_after3;
				PositionBasedPDIPCCollisionUtility::GetVertexFaceRebounce(
					vertex,
					face1, face2, face3,
					vertex_velocity,
					face_velocity1, face_velocity2, face_velocity3,
					obj1.GetCollisionVertexMass(vertex_index),
					obj2.GetCollisionVertexMass(face_indices[0]),
					obj2.GetCollisionVertexMass(face_indices[1]),
					obj2.GetCollisionVertexMass(face_indices[2]),
					local_toi, _d_hat, _velocity_damping,
					vertex_after,
					face_after1, face_after2, face_after3
				);

				const double distance2 = IPC::point_triangle_distance_unclassified(
					vertex + global_toi * vertex_velocity,
					face1 + global_toi * face_velocity1,
					face2 + global_toi * face_velocity2,
					face3 + global_toi * face_velocity3
				);

				const double stiffness = PositionBasedPDIPCCollisionUtility::GetStiffness(
					_kappa, _d_hat * _d_hat, distance2, _kappa
				);

				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id1, vertex_index, stiffness,
					vertex_after
				));
				
				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id2, face_indices[0], stiffness,
					face_after1
				));
				
				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id2, face_indices[1], stiffness,
					face_after2
				));
				
				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id2, face_indices[2], stiffness,
					face_after3
				));
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
				
				_primitive_pairs.emplace_back(primitive_pair);

				Vector3d edge_after11, edge_after12, edge_after21, edge_after22;

				const double distance2 = IPC::edge_edge_distance_unclassified(
					edge11 + global_toi * edge_velocity11,
					edge12 + global_toi * edge_velocity12,
					edge21 + global_toi * edge_velocity21,
					edge22 + global_toi * edge_velocity22
				);

				const double stiffness = PositionBasedPDIPCCollisionUtility::GetStiffness(
					_kappa, _d_hat * _d_hat, distance2, _kappa
				);
				
				PositionBasedPDIPCCollisionUtility::GetEdgeEdgeRebounce(
					edge11, edge12,
					edge21, edge22,
					edge_velocity11, edge_velocity12,
					edge_velocity21, edge_velocity22,
					obj1.GetCollisionVertexMass(edge_indices1[0]),
					obj1.GetCollisionVertexMass(edge_indices1[1]),
					obj2.GetCollisionVertexMass(edge_indices2[0]),
					obj2.GetCollisionVertexMass(edge_indices2[1]),
					local_toi, _d_hat, _velocity_damping,
					edge_after11, edge_after12,
					edge_after21, edge_after22
				);

				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id1, edge_indices1[0], stiffness,
					edge_after11
				));
				
				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id1, edge_indices1[1], stiffness,
					edge_after12
				));
				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id2, edge_indices2[0], stiffness,
					edge_after21
				));
				_projection_infos.emplace_back(ProjectionInfo(
					primitive_pair._obj_id2, edge_indices2[1], stiffness,
					edge_after22
				));

				break;
			}
		}
	}
}

double PositionBasedPDIPCCollisionHandler::GetBarrierEnergy(
	const std::vector<MassedCollisionInterface> &objs
) const {
	double energy = 0;
	for (const auto& projection_info : _projection_infos) {
		Vector3d vertex = objs[projection_info._obj_id]
						  .GetCollisionVertices()
						  .row(projection_info._vertex_id);
		
		energy += 0.5 * projection_info._stiffness *
				  (vertex - projection_info._target_position).squaredNorm();
	}
	return energy;
}

void PositionBasedPDIPCCollisionHandler::BarrierLocalProject(
	const std::vector<MassedCollisionInterface> &objs,
	const std::vector<int>& offsets,
	Ref<VectorXd> y
) const {
	// TODO: this only works for sampled objects
	for (const auto& projection_info : _projection_infos) {
		const auto& obj = objs[projection_info._obj_id];
		y.segment<3>(offsets[projection_info._obj_id] + 3 * projection_info._vertex_id) += projection_info._stiffness * projection_info._target_position;
		// obj.GetCollisionVertexDerivative(projection_info._vertex_id)
		// .RightProduct(
		// 	projection_info._stiffness * projection_info._target_position,
		// 	y.segment(offsets[projection_info._obj_id], obj.GetDOF())
		// );
	}
}

void PositionBasedPDIPCCollisionHandler::GetBarrierGlobalMatrix(
	const std::vector<MassedCollisionInterface> &objs,
	const std::vector<int>& offsets,
	COO& coo, int x_offset, int y_offset
) const {
	// TODO: this only works for sampled objects
	for (const auto& projection_info : _projection_infos) {
		const auto& obj = objs[projection_info._obj_id];
		for (int i = 0; i < 3; i++) {
			coo.push_back(Tripletd(
				x_offset + offsets[projection_info._obj_id] + 3 * projection_info._vertex_id + i,
				y_offset + offsets[projection_info._obj_id] + 3 * projection_info._vertex_id + i,
				projection_info._stiffness
			));
		}
		// obj.GetCollisionVertexDerivative(projection_info._vertex_id)
		// .RightTransposeProduct(
		// 	obj.GetCollisionVertexDerivative(projection_info._vertex_id)
		// )
		// .ToSparse(
		// 	projection_info._stiffness, coo,
		// 	x_offset + offsets[projection_info._obj_id],
		// 	y_offset + offsets[projection_info._obj_id]
		// );
	}
}

void PositionBasedPDIPCCollisionHandler::GetBarrierGlobalMatrix(
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
