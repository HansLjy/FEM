#include "PDIPCCollisionHelper.hpp"
#include "Collision/CollisionUtil/ProcessPrimitivePair.hpp"
#include "Collision/GeometryComputation.h"
#include "GeometryUtil.hpp"
#include "Collision/CollisionUtility.h"

void PDIPCCollisionUtility::GetVertexFaceRebounce(
	const Vector3d &vertex,
	const Vector3d &face1, const Vector3d &face2, const Vector3d &face3,
	const Vector3d &vertex_velocity, const Vector3d &face_velocity1, const Vector3d &face_velocity2, const Vector3d &face_velocity3,
	const double vertex_mass,
	const double face_mass1, const double face_mass2, const double face_mass3,
	const double toi,
	const double d_hat,
	Vector3d &vertex_after,
	Vector3d &face_after1, Vector3d &face_after2, Vector3d &face_after3
) {
	const Vector3d vertex_toi = vertex + vertex_velocity * toi;
	const Vector3d face_toi1 = face1 + face_velocity1 * toi;
	const Vector3d face_toi2 = face2 + face_velocity2 * toi;
	const Vector3d face_toi3 = face3 + face_velocity3 * toi;
	Vector3d barycentric_coord, normal;
	LeiLan::VertexTriangleDistance(vertex_toi, face_toi1, face_toi2, face_toi3, barycentric_coord(0), barycentric_coord(1), barycentric_coord(2), normal);
	// barycentric_coord.segment<2>(1) = GeometryUtil::GetPointPlaneClosestPoint(
	// 	vertex_toi,
	// 	face_toi1, face_toi2, face_toi3
	// );
	// barycentric_coord(0) = 1 - barycentric_coord(1) - barycentric_coord(2);
	const double m1 = vertex_mass;
	const double m2 = face_mass1 * barycentric_coord(0)
					+ face_mass2 * barycentric_coord(1)
					+ face_mass3 * barycentric_coord(2);
	// const Vector3d closest_point = barycentric_coord(0) * face_toi1
	// 							 + barycentric_coord(1) * face_toi2
	// 							 + barycentric_coord(2) * face_toi3;
	const Vector3d closest_point_velocity = barycentric_coord(0) * face_velocity1
										  + barycentric_coord(1) * face_velocity2
										  + barycentric_coord(2) * face_velocity3;
	// const Vector3d normal = (vertex_toi - closest_point).normalized();
	// std::cerr << "normal: "<< normal.transpose() << std::endl;
	double normal_velocity1 = vertex_velocity.dot(normal), normal_velocity_after1;
	double normal_velocity2 = closest_point_velocity.dot(normal), normal_velocity_after2;
	PDIPCCollisionUtility::GetPointPointRebounce(
		normal_velocity1 - normal_velocity2,
		m1, m2,
		normal_velocity_after1, normal_velocity_after2
	);
	double delta_v = std::abs(normal_velocity_after1 - normal_velocity_after2);
	double delta_v_max = d_hat / (1 - toi);
	normal_velocity_after1 *= delta_v_max / delta_v;
	normal_velocity_after2 *= delta_v_max / delta_v;
	const Vector3d vertex_tangent_velocity = vertex_velocity - vertex_velocity.dot(normal) * normal;
	const Vector3d face_tangent_velocity1 = face_velocity1 - face_velocity1.dot(normal) * normal;
	const Vector3d face_tangent_velocity2 = face_velocity2 - face_velocity2.dot(normal) * normal;
	const Vector3d face_tangent_velocity3 = face_velocity3 - face_velocity3.dot(normal) * normal;
	const Vector3d vertex_velocity_after = vertex_tangent_velocity * 0.7 + normal_velocity_after1 * normal;
	const Vector3d face_velocity_after1 = face_tangent_velocity1 * 0.7 + normal_velocity_after2 * normal;
	const Vector3d face_velocity_after2 = face_tangent_velocity2 * 0.7 + normal_velocity_after2 * normal;
	const Vector3d face_velocity_after3 = face_tangent_velocity3 * 0.7 + normal_velocity_after2 * normal;

	vertex_after = vertex + toi * vertex_velocity + (1 - toi) * vertex_velocity_after;
	face_after1 = face1 + toi * face_velocity1 + (1 - toi) * face_velocity_after1;
	face_after2 = face2 + toi * face_velocity2 + (1 - toi) * face_velocity_after2;
	face_after3 = face3 + toi * face_velocity3 + (1 - toi) * face_velocity_after3;
}

void PDIPCCollisionUtility::GetEdgeEdgeRebounce(
	const Vector3d &edge11, const Vector3d &edge12,
	const Vector3d &edge21, const Vector3d &edge22,
	const Vector3d &edge_velocity11, const Vector3d &edge_velocity12,
	const Vector3d &edge_velocity21, const Vector3d &edge_velocity22,
	const double edge_mass11, const double edge_mass12,
	const double edge_mass21, const double edge_mass22,
	const double toi,
	const double d_hat,
	Vector3d &edge_after11, Vector3d &edge_after12,
	Vector3d &edge_after21, Vector3d &edge_after22
) {
	const Vector3d edge_toi11 = edge11 + edge_velocity11 * toi;
	const Vector3d edge_toi12 = edge12 + edge_velocity12 * toi;
	const Vector3d edge_toi21 = edge21 + edge_velocity21 * toi;
	const Vector3d edge_toi22 = edge22 + edge_velocity22 * toi;

	Vector2d barycentric_coord;
	Vector3d normal;
	LeiLan::EdgeEdgeSqDistance(
		edge_toi11, edge_toi12, edge_toi21, edge_toi22,
		barycentric_coord(0), barycentric_coord(1), normal
	);

	// barycentric_coord = GeometryUtil::GetLineLineClosestPoint(
	// 	edge_toi11, edge_toi12,
	// 	edge_toi21, edge_toi22
	// );

	const double m1 = edge_mass11 * (1 - barycentric_coord(0))
					+ edge_mass12 * barycentric_coord(0);
	const double m2 = edge_mass21 * (1 - barycentric_coord(1))
					+ edge_mass22 * barycentric_coord(1);
	// const Vector3d closest_point1 = barycentric_coord(0) * edge_toi12
	// 							  + (1 - barycentric_coord(0)) * edge_toi11;
	// const Vector3d closest_point2 = barycentric_coord(1) * edge_toi22
	// 							  + (1 - barycentric_coord(1)) * edge_toi21;
	const Vector3d closest_point_velocity1 = barycentric_coord(0) * edge_velocity12
										   + (1 - barycentric_coord(0)) * edge_velocity11;
	const Vector3d closest_point_velocity2 = barycentric_coord(1) * edge_velocity22
										   + (1 - barycentric_coord(1)) * edge_velocity21;
	
	// const Vector3d normal = (closest_point1 - closest_point2).normalized();
	double normal_velocity1 = closest_point_velocity1.dot(normal), normal_velocity_after1;
	double normal_velocity2 = closest_point_velocity2.dot(normal), normal_velocity_after2;
	PDIPCCollisionUtility::GetPointPointRebounce(
		normal_velocity1 - normal_velocity2,
		m1, m2,
		normal_velocity_after1, normal_velocity_after2
	);
	double delta_v = std::abs(normal_velocity_after1 - normal_velocity_after2);
	double delta_v_max = d_hat / (1 - toi);
	normal_velocity_after1 *= delta_v_max / delta_v;
	normal_velocity_after2 *= delta_v_max / delta_v;

	const Vector3d edge_tangent_velocity11 = edge_velocity11 - edge_velocity11.dot(normal) * normal;
	const Vector3d edge_tangent_velocity12 = edge_velocity12 - edge_velocity12.dot(normal) * normal;
	const Vector3d edge_tangent_velocity21 = edge_velocity21 - edge_velocity21.dot(normal) * normal;
	const Vector3d edge_tangent_velocity22 = edge_velocity22 - edge_velocity22.dot(normal) * normal;

	const Vector3d edge_velocity_after11 = edge_tangent_velocity11 * 0.7 + normal_velocity_after1 * normal;
	const Vector3d edge_velocity_after12 = edge_tangent_velocity12 * 0.7 + normal_velocity_after1 * normal;
	const Vector3d edge_velocity_after21 = edge_tangent_velocity21 * 0.7 + normal_velocity_after2 * normal;
	const Vector3d edge_velocity_after22 = edge_tangent_velocity22 * 0.7 + normal_velocity_after2 * normal;
	edge_after11 = edge11 + toi * edge_velocity11 + (1 - toi) * edge_velocity_after11;
	edge_after12 = edge12 + toi * edge_velocity12 + (1 - toi) * edge_velocity_after12;
	edge_after21 = edge21 + toi * edge_velocity21 + (1 - toi) * edge_velocity_after21;
	edge_after22 = edge22 + toi * edge_velocity22 + (1 - toi) * edge_velocity_after22;
}

void PDIPCCollisionUtility::GetPointPointRebounce(
	double delta_v,
	double mass1, double mass2,
	double& v_after1, double& v_after2
) {
	if (mass1 < 0) {
		v_after1 = 0;
		v_after2 = delta_v;
	} else if (mass2 < 0) {
		v_after1 = - delta_v;
		v_after2 = 0;
	} else {
		v_after1 = - mass2 * delta_v / (mass1 + mass2);
		v_after2 = mass1 * delta_v / (mass1 + mass2);
	}
}

double PDIPCCollisionUtility::GetStiffness(double kappa, double d_hat, double distance) {
	if (distance >= d_hat) {
		distance = 0.99 * d_hat;
	}
	distance = std::max(1e-8 * d_hat, distance);
	return - kappa * std::log(distance / d_hat);
}

void PDIPCCollisionHandler::ClearBarrierSet() {
	_barrier_infos.clear();
}

void PDIPCCollisionHandler::AddBarrierPairs(
	const std::vector<CollisionInterface> &objs,
	const std::vector<int> &offsets,
	const std::vector<PrimitivePair> &barrier_candidate_set
) {
	for (const auto& barrier_pair : barrier_candidate_set) {
		const auto& obj1 = objs[barrier_pair._obj_id1];
		const auto& obj2 = objs[barrier_pair._obj_id2];
		bool is_barrier = false;
		double distance;
		Vector4d barycentric_coords;
		Vector3d normal;
		switch (barrier_pair._type) {
			case CollisionType::kVertexFace: {
				const int vertex_index = barrier_pair._primitive_id1;
				const Vector3d vertex_toi = obj1.GetCollisionVertices().row(vertex_index).transpose();

				const RowVector3i face_indices = obj2.GetCollisionFaceTopo().row(barrier_pair._primitive_id2);
				const Vector3d face_toi1 = obj2.GetCollisionVertices().row(face_indices[0]).transpose();
				const Vector3d face_toi2 = obj2.GetCollisionVertices().row(face_indices[1]).transpose();
				const Vector3d face_toi3 = obj2.GetCollisionVertices().row(face_indices[2]).transpose();

				distance = GetVFDistance(vertex_toi, face_toi1, face_toi2, face_toi3);

				if (distance < _d_hat) {
					auto compact_barycentric_coords = GeometryUtil::GetPointPlaneClosestPoint (
						vertex_toi, face_toi1, face_toi2, face_toi3
					);

					const Vector3d closest_point_on_face = face_toi1
														+ compact_barycentric_coords[0] * (face_toi2 - face_toi1)
														+ compact_barycentric_coords[1] * (face_toi3 - face_toi1);
					
					is_barrier = true;
					normal = (vertex_toi - closest_point_on_face).normalized();
					barycentric_coords << 1, 1 - compact_barycentric_coords[0] - compact_barycentric_coords[1],
										compact_barycentric_coords[0], compact_barycentric_coords[1];
				}


				break;
			}
			case CollisionType::kEdgeEdge: {
				const RowVector2i edge_indices1 = obj1.GetCollisionEdgeTopo().row(barrier_pair._primitive_id1);
				const Vector3d edge_toi11 = obj1.GetCollisionVertices().row(edge_indices1[0]).transpose();
				const Vector3d edge_toi12 = obj1.GetCollisionVertices().row(edge_indices1[1]).transpose();

				const RowVector2i edge_indices2 = obj2.GetCollisionEdgeTopo().row(barrier_pair._primitive_id2);
				const Vector3d edge_toi21 = obj2.GetCollisionVertices().row(edge_indices2[0]).transpose();
				const Vector3d edge_toi22 = obj2.GetCollisionVertices().row(edge_indices2[1]).transpose();

				distance = GetEEDistance(edge_toi11, edge_toi12, edge_toi21, edge_toi22);

				if (distance < _d_hat) {
					auto compact_barycentric_coords = GeometryUtil::GetLineLineClosestPoint(
						edge_toi11, edge_toi12, edge_toi21, edge_toi22
					);

					const Vector3d closest_point_on_edge1 = edge_toi11 + compact_barycentric_coords[0] * (edge_toi12 - edge_toi11);
					const Vector3d closest_point_on_edge2 = edge_toi21 + compact_barycentric_coords[1] * (edge_toi22 - edge_toi21);

					is_barrier = true;
					normal = (closest_point_on_edge1 - closest_point_on_edge2).normalized();
					barycentric_coords << 1 - compact_barycentric_coords[0], compact_barycentric_coords[0],
										  1 - compact_barycentric_coords[1], compact_barycentric_coords[1];
				}

				break;
			}
		}
		if (is_barrier) {
			_barrier_infos.push_back({
				barrier_pair._type,
				barrier_pair._obj_id1,
				barrier_pair._obj_id2,
				barrier_pair._primitive_id1,
				barrier_pair._primitive_id2,
				barycentric_coords,
				normal,
				PDIPCCollisionUtility::GetStiffness(_kappa, _d_hat, distance)
			});
		}
	}
}

double PDIPCCollisionHandler::GetBarrierEnergy(
	const std::vector<CollisionInterface> &objs
) const {
	double energy = 0;
	for (const auto& barrier_pair : _barrier_infos) {
		const auto& obj1 = objs[barrier_pair._obj_id1];
		const auto& obj2 = objs[barrier_pair._obj_id2];
		switch (barrier_pair._type) {
			case CollisionType::kVertexFace: {
				int vertex_index = barrier_pair._primitive_id1;
				const Vector3d vertex = obj1.GetCollisionVertices().row(vertex_index);

				const RowVector3i face_indices = obj2.GetCollisionFaceTopo().row(barrier_pair._primitive_id2);
				const Vector3d face1 = obj2.GetCollisionVertices().row(face_indices[0]);
				const Vector3d face2 = obj2.GetCollisionVertices().row(face_indices[1]);
				const Vector3d face3 = obj2.GetCollisionVertices().row(face_indices[2]);

				double d = barrier_pair._normal.dot(
					vertex - barrier_pair._barycentric_coords[1] * face1
						   - barrier_pair._barycentric_coords[2] * face2
						   - barrier_pair._barycentric_coords[3] * face3
				);

				if (d < _d_hat) {
					energy += 0.5 * barrier_pair._stiffness * (_d_hat - d) * (_d_hat - d);
				}
				break;
			}
			case CollisionType::kEdgeEdge: {
				const RowVector2i edge_indices1 = obj1.GetCollisionEdgeTopo().row(barrier_pair._primitive_id1);
				const Vector3d edge11 = obj1.GetCollisionVertices().row(edge_indices1[0]);
				const Vector3d edge12 = obj1.GetCollisionVertices().row(edge_indices1[1]);

				const RowVector2i edge_indices2 = obj2.GetCollisionEdgeTopo().row(barrier_pair._primitive_id2);
				const Vector3d edge21 = obj2.GetCollisionVertices().row(edge_indices2[0]);
				const Vector3d edge22 = obj2.GetCollisionVertices().row(edge_indices2[1]);

				double d = barrier_pair._normal.dot(
					barrier_pair._barycentric_coords[0] * edge11
					+ barrier_pair._barycentric_coords[1] * edge12
					- barrier_pair._barycentric_coords[2] * edge21
					- barrier_pair._barycentric_coords[3] * edge22
				);

				if (d < _d_hat) {
					energy += 0.5 * barrier_pair._stiffness * (_d_hat - d) * (_d_hat - d);
				}
				break;
			}
		}
	}
	return energy;
}

void PDIPCCollisionHandler::BarrierLocalProject(
	const std::vector<CollisionInterface> &objs,
	const std::vector<int>& offsets,
	Ref<VectorXd> y
) const {
	for (const auto& barrier_pair : _barrier_infos) {
		const auto& obj1 = objs[barrier_pair._obj_id1];
		const auto& obj2 = objs[barrier_pair._obj_id2];
		Ref<VectorXd> y1 = y.segment(offsets[barrier_pair._obj_id1], obj1.GetDOF());
		Ref<VectorXd> y2 = y.segment(offsets[barrier_pair._obj_id2], obj2.GetDOF());

		switch (barrier_pair._type) {
			case CollisionType::kVertexFace: {
				int vertex_index = barrier_pair._primitive_id1;
				const Vector3d vertex = obj1.GetCollisionVertices().row(vertex_index);

				const RowVector3i face_indices = obj2.GetCollisionFaceTopo().row(barrier_pair._primitive_id2);
				const Vector3d face1 = obj2.GetCollisionVertices().row(face_indices[0]);
				const Vector3d face2 = obj2.GetCollisionVertices().row(face_indices[1]);
				const Vector3d face3 = obj2.GetCollisionVertices().row(face_indices[2]);

				double d = barrier_pair._normal.dot(
					vertex - barrier_pair._barycentric_coords[1] * face1
						   - barrier_pair._barycentric_coords[2] * face2
						   - barrier_pair._barycentric_coords[3] * face3
				);

				if (d < _d_hat) {
					d = _d_hat;
				}

				obj1.GetCollisionVertexDerivative(vertex_index)
				.RightProduct(
					barrier_pair._stiffness * barrier_pair._normal * d, y1
				);

				obj2.GetCollisionVertexDerivative(face_indices[0])
				.RightProduct(
					- barrier_pair._stiffness * barrier_pair._barycentric_coords[1] * d * barrier_pair._normal, y2
				);

				obj2.GetCollisionVertexDerivative(face_indices[1])
				.RightProduct(
					- barrier_pair._stiffness * barrier_pair._barycentric_coords[2] * d * barrier_pair._normal, y2
				);

				obj2.GetCollisionVertexDerivative(face_indices[2])
				.RightProduct(
					- barrier_pair._stiffness * barrier_pair._barycentric_coords[3] * d * barrier_pair._normal, y2
				);

				break;
			}
			case CollisionType::kEdgeEdge: {
				const RowVector2i edge_indices1 = obj1.GetCollisionEdgeTopo().row(barrier_pair._primitive_id1);
				const Vector3d edge11 = obj1.GetCollisionVertices().row(edge_indices1[0]);
				const Vector3d edge12 = obj1.GetCollisionVertices().row(edge_indices1[1]);

				const RowVector2i edge_indices2 = obj2.GetCollisionEdgeTopo().row(barrier_pair._primitive_id2);
				const Vector3d edge21 = obj2.GetCollisionVertices().row(edge_indices2[0]);
				const Vector3d edge22 = obj2.GetCollisionVertices().row(edge_indices2[1]);

				double d = barrier_pair._normal.dot(
					barrier_pair._barycentric_coords[0] * edge11
					+ barrier_pair._barycentric_coords[1] * edge12
					- barrier_pair._barycentric_coords[2] * edge21
					- barrier_pair._barycentric_coords[3] * edge22
				);

				if (d < _d_hat) {
					d = _d_hat;
				}

				obj1.GetCollisionVertexDerivative(edge_indices1[0])
				.RightProduct(
					barrier_pair._stiffness * barrier_pair._barycentric_coords[0] * d * barrier_pair._normal, y1
				);

				obj1.GetCollisionVertexDerivative(edge_indices1[1])
				.RightProduct(
					barrier_pair._stiffness * barrier_pair._barycentric_coords[1] * d * barrier_pair._normal, y1
				);

				obj2.GetCollisionVertexDerivative(edge_indices2[0])
				.RightProduct(
					- barrier_pair._stiffness * barrier_pair._barycentric_coords[2] * d * barrier_pair._normal, y2
				);

				obj2.GetCollisionVertexDerivative(edge_indices2[1])
				.RightProduct(
					- barrier_pair._stiffness * barrier_pair._barycentric_coords[3] * d * barrier_pair._normal, y2
				);
				break;
			}
		}
	}
}

void PDIPCCollisionHandler::GetBarrierGlobalMatrix(
	const std::vector<CollisionInterface> &objs,
	const std::vector<int>& offsets,
	COO& coo, int x_offset, int y_offset
) const {
	for (const auto& barrier_pair : _barrier_infos) {
		const auto& obj1 = objs[barrier_pair._obj_id1];
		const auto& obj2 = objs[barrier_pair._obj_id2];
		const int offset1 = offsets[barrier_pair._obj_id1];
		const int offset2 = offsets[barrier_pair._obj_id2];
		switch (barrier_pair._type) {
			case CollisionType::kVertexFace: {
				const int vertex_index = barrier_pair._primitive_id1;
				const RowVector3i face_indices = obj2.GetCollisionFaceTopo().row(barrier_pair._primitive_id2);

				const int submatrix_offsets[] = {
					offset1, offset2, offset2, offset2
				};

				BlockVector AS[4] = {
					obj1.GetCollisionVertexDerivative(vertex_index).RightProduct(barrier_pair._normal),
					obj2.GetCollisionVertexDerivative(face_indices[0]).RightProduct(- barrier_pair._normal * barrier_pair._barycentric_coords[1]),
					obj2.GetCollisionVertexDerivative(face_indices[1]).RightProduct(- barrier_pair._normal * barrier_pair._barycentric_coords[2]),
					obj2.GetCollisionVertexDerivative(face_indices[2]).RightProduct(- barrier_pair._normal * barrier_pair._barycentric_coords[3])
				};

				for (int i = 0; i < 4; i++) {
					for (int j = 0; j < 4; j++) {
						AS[i].RightTransposeProduct(AS[j])
						.ToSparse(
							barrier_pair._stiffness, coo,
							x_offset + submatrix_offsets[i],
							y_offset + submatrix_offsets[j]
						);
					}
				}

				break;
			}

			case CollisionType::kEdgeEdge: {
				const RowVector2i edge_indices1 = obj1.GetCollisionEdgeTopo().row(barrier_pair._primitive_id1);
				const RowVector2i edge_indices2 = obj2.GetCollisionEdgeTopo().row(barrier_pair._primitive_id2);

				const int submatrix_offsets[] = {
					offset1, offset1, offset2, offset2
				};

				BlockVector AS[4] = {
					obj1.GetCollisionVertexDerivative(edge_indices1[0]).RightProduct(barrier_pair._barycentric_coords[0] * barrier_pair._normal),
					obj1.GetCollisionVertexDerivative(edge_indices1[1]).RightProduct(barrier_pair._barycentric_coords[1] * barrier_pair._normal),
					obj2.GetCollisionVertexDerivative(edge_indices2[0]).RightProduct(- barrier_pair._barycentric_coords[2] * barrier_pair._normal),
					obj2.GetCollisionVertexDerivative(edge_indices2[1]).RightProduct(- barrier_pair._barycentric_coords[3] * barrier_pair._normal),
				};

				for (int i = 0; i < 4; i++) {
					for (int j = 0; j < 4; j++) {
						AS[i].RightTransposeProduct(AS[j])
						.ToSparse(
							barrier_pair._stiffness, coo,
							x_offset + submatrix_offsets[i],
							y_offset + submatrix_offsets[j]
						);
					}
				}
				break;
			}
		}
	}
}

SparseMatrixXd PDIPCCollisionHandler::GetBarrierGlobalMatrix(
	const std::vector<CollisionInterface> &objs,
	const std::vector<int>& offsets,
	int total_dof
) const {
	COO coo;
	GetBarrierGlobalMatrix(objs, offsets, coo, 0, 0);
	SparseMatrixXd global_matrix(total_dof, total_dof);
	global_matrix.setFromTriplets(coo.begin(), coo.end());
	return global_matrix;
}
