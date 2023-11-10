#include "PositionBasedPDIPCCollisionHelper.hpp"
#include "Collision/CollisionUtil/CCD/CubicSolver/CemCubicSolver.hpp"
#include "Collision/CollisionUtil/CCD/SimpleCCD.h"
#include "Collision/CollisionUtil/ProcessPrimitivePair.hpp"
#include "Collision/GeometryComputation.h"
#include "Collision/IpcTookit/EdgeEdgeCcd.h"
#include "Collision/IpcTookit/PointTriangleCcd.h"
#include "FileIO.hpp"
#include "GeometryUtil.hpp"
#include "Collision/IpcTookit/UnclassifiedDistance.hpp"
#include "Collision/CollisionUtility.h"
#include "Collision/CollisionInterface.hpp"

template<>
Caster<MassedCollisionInterface>* Caster<MassedCollisionInterface>::_the_factory = nullptr;

// bool debug_bit = false;

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
	Vector3d &face_after1, Vector3d &face_after2, Vector3d &face_after3,
	double& mass1, double& mass2
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

	// if (distance2 < 1e-12) {
	// 	spdlog::warn("vertex-face pair too close!");
	// }

	mass1 = vertex_mass;
	mass2 = face_mass1 * barycentric_coord(0)
		  + face_mass2 * barycentric_coord(1)
		  + face_mass3 * barycentric_coord(2);
	
	if (local_toi == 1) {
		vertex_after = vertex_toi;
		face_after1 = face_toi1;
		face_after2 = face_toi2;
		face_after3 = face_toi3;
		return;
	}

	const Vector3d closest_point_velocity = barycentric_coord(0) * face_velocity1
										  + barycentric_coord(1) * face_velocity2
										  + barycentric_coord(2) * face_velocity3;
	double normal_velocity1 = vertex_velocity.dot(normal);
	double normal_velocity2 = closest_point_velocity.dot(normal);

	const Vector3d tangent_velocity1 = vertex_velocity - normal_velocity1 * normal;
	const Vector3d tangent_velocity2 = closest_point_velocity - normal_velocity2 * normal;

	assert((tangent_velocity1 + normal_velocity1 * normal - vertex_velocity).norm() < 1e-10);
	assert((tangent_velocity2 + normal_velocity2 * normal - closest_point_velocity).norm() < 1e-10);

	const double rebounce_dist = d_hat - std::sqrt(distance2);

	const double normal_velocity_after1 =   rebounce_dist * (mass2 / (mass1 + mass2)) / (1 - local_toi);
	const double normal_velocity_after2 = - rebounce_dist * (mass1 / (mass1 + mass2)) / (1 - local_toi);


	const Vector3d velocity_after1 = normal_velocity_after1 * normal + tangent_velocity1 * velocity_damping;
	const Vector3d velocity_after2 = normal_velocity_after2 * normal + tangent_velocity2 * velocity_damping;

	vertex_after = vertex + local_toi * vertex_velocity + (1 - local_toi) * velocity_after1;
	face_after1 = face1 + local_toi * face_velocity1 + (1 - local_toi) * velocity_after2;
	face_after2 = face2 + local_toi * face_velocity2 + (1 - local_toi) * velocity_after2;
	face_after3 = face3 + local_toi * face_velocity3 + (1 - local_toi) * velocity_after2;

	// if (debug_bit) {
	// 	std::vector<Vector3d> vertices;
	// 	vertices.push_back(vertex_toi);
	// 	vertices.push_back(face_toi1);
	// 	vertices.push_back(face_toi2);
	// 	vertices.push_back(face_toi3);
	// 	DebugUtils::DumpVertexList("fuck-toi", vertices);

	// 	std::cerr << distance2 << std::endl;
	// 	std::cerr << normal.dot((face_toi2 - face_toi1).normalized()) << " " << normal.dot((face_toi3 - face_toi1).normalized()) << std::endl;

	// 	std::cerr << "==============" << std::endl;
	// 	std::cerr << normal.transpose() << std::endl
	// 			  << vertex_toi.transpose() << std::endl
	// 			  << face_toi1.transpose() << std::endl
	// 			  << face_toi2.transpose() << std::endl
	// 			  << face_toi3.transpose() << std::endl
	// 			  << normal_velocity_after1 << " " << normal_velocity_after2 << std::endl
	// 			  << tangent_velocity1.transpose() << std::endl
	// 			  << tangent_velocity2.transpose() << std::endl
	// 			  << velocity_after1.transpose() << std::endl
	// 			  << velocity_after2.transpose() << std::endl;
	// }
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
	Vector3d &edge_after21, Vector3d &edge_after22,
	double &mass1, double &mass2
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

	// if (distance2 < 1e-12) {
	// 	spdlog::warn("edge-edge pair too close!");
	// }

	mass1 = edge_mass11 * (1 - barycentric_coord(0))
		  + edge_mass12 * barycentric_coord(0);
	mass2 = edge_mass21 * (1 - barycentric_coord(1))
		  + edge_mass22 * barycentric_coord(1);
		
	if (local_toi == 1) {
		edge_after11 = edge_toi11;
		edge_after12 = edge_toi12;
		edge_after21 = edge_toi21;
		edge_after22 = edge_toi22;
		return;
	}

	const Vector3d closest_point_velocity1 = barycentric_coord(0) * edge_velocity12
										   + (1 - barycentric_coord(0)) * edge_velocity11;
	const Vector3d closest_point_velocity2 = barycentric_coord(1) * edge_velocity22
										   + (1 - barycentric_coord(1)) * edge_velocity21;
	
	double normal_velocity1 = closest_point_velocity1.dot(normal);
	double normal_velocity2 = closest_point_velocity2.dot(normal);

	const Vector3d tangent_velocity1 = closest_point_velocity1 - normal_velocity1 * normal;
	const Vector3d tangent_velocity2 = closest_point_velocity2 - normal_velocity2 * normal;

	const double rebounce_dist = d_hat - std::sqrt(distance2);

	const double normal_velocity_after1 =   rebounce_dist * (mass2 / (mass1 + mass2)) / (1 - local_toi);
	const double normal_velocity_after2 = - rebounce_dist * (mass1 / (mass1 + mass2)) / (1 - local_toi);

	const Vector3d edge_velocity_after1 = normal_velocity_after1 * normal + tangent_velocity1 * velocity_damping;
	const Vector3d edge_velocity_after2 = normal_velocity_after2 * normal + tangent_velocity2 * velocity_damping;

	edge_after11 = edge11 + local_toi * edge_velocity11 + (1 - local_toi) * edge_velocity_after1;
	edge_after12 = edge12 + local_toi * edge_velocity12 + (1 - local_toi) * edge_velocity_after1;
	edge_after21 = edge21 + local_toi * edge_velocity21 + (1 - local_toi) * edge_velocity_after2;
	edge_after22 = edge22 + local_toi * edge_velocity22 + (1 - local_toi) * edge_velocity_after2;
}

double PositionBasedPDIPCCollisionUtility::GetStiffness(
	double kappa, double d_hat2, double distance2, double mass, double dt
) {
    if (distance2 < d_hat2) {
		double t2 = distance2 - d_hat2;
        return -kappa * (t2 / d_hat2) * (t2 / d_hat2) * std::log(distance2 / d_hat2) + 1e3 * mass / (dt * dt);
    } else {
		return 0;
	}
}

void PositionBasedPDIPCCollisionHandler::ClearConstraintSet() {
	_barrier_set.clear();
	_projection_infos.clear();
}

void PositionBasedPDIPCCollisionHandler::AddBarrierSet(
	const std::vector<PrimitivePair> barrier_set
) {
	for (const auto& barrier_pair : barrier_set) {
		_barrier_set.insert(barrier_pair);
	}
}

void PositionBasedPDIPCCollisionHandler::TargetPositionGeneration(
	const std::vector<MassedCollisionInterface> &objs,
	double global_toi, double dt
) {
	_projection_infos.clear();
	_projection_infos.reserve(_barrier_set.size() * 4);
	int active_c_set = 0;
	for (const auto& barrier_primitive_pair : _barrier_set) {
		const auto& obj1 = objs[barrier_primitive_pair._obj_id1];
		const auto& obj2 = objs[barrier_primitive_pair._obj_id2];
		
		switch (barrier_primitive_pair._type) {
			case CollisionType::kVertexFace: {
				const int vertex_index = barrier_primitive_pair._primitive_id1;
				const Vector3d vertex = obj1.GetCollisionVertices().row(vertex_index);
				const Vector3d vertex_velocity = obj1.GetCollisionVertexVelocity(vertex_index);
				
				const RowVector3i face_indices = obj2.GetCollisionFaceTopo().row(barrier_primitive_pair._primitive_id2);
				const Vector3d face1 = obj2.GetCollisionVertices().row(face_indices[0]);
				const Vector3d face2 = obj2.GetCollisionVertices().row(face_indices[1]);
				const Vector3d face3 = obj2.GetCollisionVertices().row(face_indices[2]);
				const Vector3d face_velocity1 = obj2.GetCollisionVertexVelocity(face_indices[0]);
				const Vector3d face_velocity2 = obj2.GetCollisionVertexVelocity(face_indices[1]);
				const Vector3d face_velocity3 = obj2.GetCollisionVertexVelocity(face_indices[2]);

				Vector3d vertex_after, face_after1, face_after2, face_after3;

				const double distance2 = IPC::point_triangle_distance_unclassified(
					vertex + global_toi * vertex_velocity,
					face1 + global_toi * face_velocity1,
					face2 + global_toi * face_velocity2,
					face3 + global_toi * face_velocity3
				);

				if (distance2 >= _d_hat * _d_hat) {
					break;
				}
				
				active_c_set++;

				double local_toi = IPC::point_triangle_ccd(
					vertex, face1, face2, face3, 
					vertex_velocity, face_velocity1, face_velocity2, face_velocity3, 
					0.1, 0
				);

				if (local_toi > 1) {
					local_toi = 1;
				} else {
					local_toi *= 0.8;
				}

				double mass1, mass2;
				PositionBasedPDIPCCollisionUtility::GetVertexFaceRebounce(
					vertex, face1, face2, face3,
					vertex_velocity, face_velocity1, face_velocity2, face_velocity3,
					obj1.GetCollisionVertexMass(vertex_index),
					obj2.GetCollisionVertexMass(face_indices[0]),
					obj2.GetCollisionVertexMass(face_indices[1]),
					obj2.GetCollisionVertexMass(face_indices[2]),
					global_toi, _d_hat, _velocity_damping,
					vertex_after, face_after1, face_after2, face_after3,
					mass1, mass2
				);

				const double stiffness1 = PositionBasedPDIPCCollisionUtility::GetStiffness(
					_kappa, _d_hat * _d_hat, distance2, mass1, dt
				);

				const double stiffness2 = PositionBasedPDIPCCollisionUtility::GetStiffness(
					_kappa, _d_hat * _d_hat, distance2, mass2, dt
				);

				_projection_infos.emplace_back(ProjectionInfo(
					barrier_primitive_pair._obj_id1, vertex_index, stiffness1,
					vertex_after
				));
				
				_projection_infos.emplace_back(ProjectionInfo(
					barrier_primitive_pair._obj_id2, face_indices[0], stiffness2,
					face_after1
				));
				
				_projection_infos.emplace_back(ProjectionInfo(
					barrier_primitive_pair._obj_id2, face_indices[1], stiffness2,
					face_after2
				));
				
				_projection_infos.emplace_back(ProjectionInfo(
					barrier_primitive_pair._obj_id2, face_indices[2], stiffness2,
					face_after3
				));

				// if (vertex_index == 3280 || face_indices[0] == 3280 || face_indices[1] == 3280 || face_indices[2] == 3280) {
				// 	DebugUtils::PrintPrimitivePair(barrier_primitive_pair, objs);
				// }

				// if (DebugUtils::PrimitivePairEqual(barrier_primitive_pair, 0, 3965, 1, 1)) {
				// 	// debug_targets.push_back(vertex_after);
				// 	debug_bit = true;
				// 	PositionBasedPDIPCCollisionUtility::GetVertexFaceRebounce(
				// 		vertex, face1, face2, face3,
				// 		vertex_velocity, face_velocity1, face_velocity2, face_velocity3,
				// 		obj1.GetCollisionVertexMass(vertex_index),
				// 		obj2.GetCollisionVertexMass(face_indices[0]),
				// 		obj2.GetCollisionVertexMass(face_indices[1]),
				// 		obj2.GetCollisionVertexMass(face_indices[2]),
				// 		local_toi, _d_hat, _velocity_damping,
				// 		vertex_after, face_after1, face_after2, face_after3,
				// 		mass1, mass2
				// 	);

				// 	debug_bit = false;
				// }

				break;
			}
			case CollisionType::kEdgeEdge: {
				const RowVector2i edge_indices1 = obj1.GetCollisionEdgeTopo().row(barrier_primitive_pair._primitive_id1);
				const Vector3d edge11 = obj1.GetCollisionVertices().row(edge_indices1[0]);
				const Vector3d edge12 = obj1.GetCollisionVertices().row(edge_indices1[1]);
				const Vector3d edge_velocity11 = obj1.GetCollisionVertexVelocity(edge_indices1[0]);
				const Vector3d edge_velocity12 = obj1.GetCollisionVertexVelocity(edge_indices1[1]);
				
				const RowVector2i edge_indices2 = obj2.GetCollisionEdgeTopo().row(barrier_primitive_pair._primitive_id2);
				const Vector3d edge21 = obj2.GetCollisionVertices().row(edge_indices2[0]);
				const Vector3d edge22 = obj2.GetCollisionVertices().row(edge_indices2[1]);
				const Vector3d edge_velocity21 = obj2.GetCollisionVertexVelocity(edge_indices2[0]);
				const Vector3d edge_velocity22 = obj2.GetCollisionVertexVelocity(edge_indices2[1]);
				

				Vector3d edge_after11, edge_after12, edge_after21, edge_after22;

				const double distance2 = IPC::edge_edge_distance_unclassified(
					edge11 + global_toi * edge_velocity11,
					edge12 + global_toi * edge_velocity12,
					edge21 + global_toi * edge_velocity21,
					edge22 + global_toi * edge_velocity22
				);

				if (distance2 >= _d_hat * _d_hat) {
					break;
				}
				active_c_set++;

				double local_toi = IPC::edge_edge_ccd(
					edge11, edge12, edge21, edge22, 
					edge_velocity11, edge_velocity12, edge_velocity21, edge_velocity22, 
					0.1, 0
				);

				if (local_toi > 1) {
					local_toi = 1;
				} else {
					local_toi *= 0.8;
				}
				
				double mass1, mass2;

				PositionBasedPDIPCCollisionUtility::GetEdgeEdgeRebounce(
					edge11, edge12, edge21, edge22,
					edge_velocity11, edge_velocity12, edge_velocity21, edge_velocity22,
					obj1.GetCollisionVertexMass(edge_indices1[0]),
					obj1.GetCollisionVertexMass(edge_indices1[1]),
					obj2.GetCollisionVertexMass(edge_indices2[0]),
					obj2.GetCollisionVertexMass(edge_indices2[1]),
					global_toi, _d_hat, _velocity_damping,
					edge_after11, edge_after12, edge_after21, edge_after22,
					mass1, mass2
				);

				const double stiffness1 = PositionBasedPDIPCCollisionUtility::GetStiffness(
					_kappa, _d_hat * _d_hat, distance2, mass1, dt
				);

				const double stiffness2 = PositionBasedPDIPCCollisionUtility::GetStiffness(
					_kappa, _d_hat * _d_hat, distance2, mass2, dt
				);

				_projection_infos.emplace_back(ProjectionInfo(
					barrier_primitive_pair._obj_id1, edge_indices1[0], stiffness1,
					edge_after11
				));
				
				_projection_infos.emplace_back(ProjectionInfo(
					barrier_primitive_pair._obj_id1, edge_indices1[1], stiffness1,
					edge_after12
				));
				_projection_infos.emplace_back(ProjectionInfo(
					barrier_primitive_pair._obj_id2, edge_indices2[0], stiffness2,
					edge_after21
				));
				_projection_infos.emplace_back(ProjectionInfo(
					barrier_primitive_pair._obj_id2, edge_indices2[1], stiffness2,
					edge_after22
				));

				// if (edge_indices1[0] == 3280 || edge_indices1[1] == 3280 || edge_indices2[0] == 3280 || edge_indices2[1] == 3280) {
				// 	DebugUtils::PrintPrimitivePair(barrier_primitive_pair, objs);
				// }

				break;
			}
		}
	}
	// spdlog::info("current active primitive pairs = {}", active_c_set);

	// if (global_toi < 1e-5) {
	// 	std::vector<Vector3d> targets;
	// 	for (const auto& projection_info : _projection_infos) {
	// 		if (projection_info._obj_id == 0 && projection_info._vertex_id == 3280) {
	// 			targets.push_back(projection_info._target_position);
	// 			std::cerr << projection_info._stiffness << std::endl;
	// 		}
	// 	}
	// 	DebugUtils::DumpVertexList("fuck-bb", targets);
	// 	exit(-1);
	// }

	// DebugUtils::DumpVertexList("fuck-bb", debug_targets);


	// std::cerr << "===========" << std::endl;
	// for (const auto& projection_info : _projection_infos) {
	// 	std::cerr << "obj id: " << projection_info._obj_id << std::endl
	// 			  << "vertex id: " << projection_info._vertex_id << std::endl
	// 			  << "stiffness: " << projection_info._stiffness << std::endl
	// 			  << "target position: " << projection_info._target_position.transpose() << std::endl;
	// }
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
