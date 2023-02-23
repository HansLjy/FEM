//
// Created by hansljy on 12/2/22.
//

#include "IPCBarrierTarget.h"
#include "Collision/CollisionUtility.h"
#include "spdlog/spdlog.h"
#include "Collision/CollisionShape/CollisionShape.h"

IPCBarrierTarget::IPCBarrierTarget(const std::vector<Object*>& objs, int begin, int end, const json& config)
    : Target(objs, begin, end, config),
      _d_hat(config["d-hat"]),
	  _kappa(config["kappa"]),
      _ccd(CCDFactory::GetCCD(config["ccd"]["type"], config["ccd"])),
	  _edge_hash_table(config["hashing"]["grid-size"], config["hashing"]["hash-table-size"]),
	  _vertex_hash_table(config["hashing"]["grid-size"], config["hashing"]["hash-table-size"]) {}

double IPCBarrierTarget::GetPotentialEnergy(const Ref<const Eigen::VectorXd> &x) const {
	return Target::GetPotentialEnergy(x) + GetBarrierEnergy();
}

void IPCBarrierTarget::GetPotentialEnergyGradient(const Ref<const Eigen::VectorXd> &x, Ref<Eigen::VectorXd> gradient) const {
	Target::GetPotentialEnergyGradient(x, gradient);
	gradient += GetBarrierEnergyGradient();
}

void IPCBarrierTarget::GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const {
	Target::GetPotentialEnergyHessian(x, coo, offset_x, offset_y);
	GetBarrierEnergyHessian(coo, offset_x, offset_y);
}

#define PROCESS_CONSTRAINT_PAIR(Initialization, VF, EE) \
    for (const auto& constraint_pair : _constraint_set) {\
        const auto obj1 = _objs[constraint_pair._obj_id1];\
        const auto obj2 = _objs[constraint_pair._obj_id2];\
        \
		const auto& shape1 = obj1->_collision_shape;\
		const auto& shape2 = obj2->_collision_shape;\
		\
        const auto& vertices1 = shape1->GetCollisionVertices();\
        const auto& vertices2 = shape2->GetCollisionVertices();\
		\
		const auto& rotation1 = obj1->GetFrameRotation();\
		const auto& rotation2 = obj2->GetFrameRotation();\
		const auto& translation1 = obj1->GetFrameX();\
		const auto& translation2 = obj2->GetFrameX();\
        \
        Initialization \
        switch (constraint_pair._type) {\
            case CollisionType::kVertexFace: {\
                const int vertex_index = constraint_pair._primitive_id1;\
                const RowVector3i face_index = shape2->GetCollisionFaceTopo().row(constraint_pair._primitive_id2);\
                const Vector3d\
                    vertex = vertices1.row(vertex_index),\
                    face1 = vertices2.row(face_index(0)),\
                    face2 = vertices2.row(face_index(1)),\
                    face3 = vertices2.row(face_index(2));\
                VF \
                break;\
            }\
            case CollisionType::kEdgeEdge: {\
                const RowVector2i\
                    edge_index1 = shape1->GetCollisionEdgeTopo().row(constraint_pair._primitive_id1),\
                    edge_index2 = shape2->GetCollisionEdgeTopo().row(constraint_pair._primitive_id2);\
                const Vector3d\
                    edge11 = vertices1.row(edge_index1(0)),\
                    edge12 = vertices1.row(edge_index1(1)),\
                    edge21 = vertices2.row(edge_index2(0)),\
                    edge22 = vertices2.row(edge_index2(1));\
                EE \
                break;\
            }\
        }\
    }

double IPCBarrierTarget::GetBarrierEnergy() const {
    double barrier_energy = 0;
    PROCESS_CONSTRAINT_PAIR(
        ,
        barrier_energy += GetVFBarrierEnergy(vertex, face1, face2, face3);,
        barrier_energy += GetEEBarrierEnergy(edge11, edge12, edge21, edge22);
    )
    return barrier_energy * _kappa;
}

VectorXd IPCBarrierTarget::GetBarrierEnergyGradient() const {
    VectorXd gradient(GetDOF());
    gradient.setZero();

    PROCESS_CONSTRAINT_PAIR (
        const int dof1 = obj1->GetDOF();
        const int dof2 = obj2->GetDOF();
        const int offset1 = _offsets[constraint_pair._obj_id1];
        const int offset2 = _offsets[constraint_pair._obj_id2];,

        const Vector12d single_gradient = GetVFBarrierEnergyGradient(vertex, face1, face2, face3);
        shape1->GetVertexDerivative(vertex_index).RightProduct(single_gradient.segment<3>(0), gradient.segment(offset1, dof1));
        shape2->GetVertexDerivative(face_index(0)).RightProduct(single_gradient.segment<3>(3), gradient.segment(offset2, dof2));
        shape2->GetVertexDerivative(face_index(1)).RightProduct(single_gradient.segment<3>(6), gradient.segment(offset2, dof2));
        shape2->GetVertexDerivative(face_index(2)).RightProduct(single_gradient.segment<3>(9), gradient.segment(offset2, dof2));
        ,

        const Vector12d single_gradient = GetEEBarrierEnergyGradient(edge11, edge12, edge21, edge22);
        shape1->GetVertexDerivative(edge_index1(0)).RightProduct(single_gradient.segment<3>(0), gradient.segment(offset1, dof1));
        shape1->GetVertexDerivative(edge_index1(1)).RightProduct(single_gradient.segment<3>(3), gradient.segment(offset1, dof1));
        shape2->GetVertexDerivative(edge_index2(0)).RightProduct(single_gradient.segment<3>(6), gradient.segment(offset2, dof2));
        shape2->GetVertexDerivative(edge_index2(1)).RightProduct(single_gradient.segment<3>(9), gradient.segment(offset2, dof2));
    )

    return gradient * _kappa;
}

void IPCBarrierTarget::GetBarrierEnergyHessian(COO &coo, int offset_x, int offset_y) const {
    for (const auto& constraint_pair : _constraint_set) {
        const auto obj1 = _objs[constraint_pair._obj_id1];
        const auto obj2 = _objs[constraint_pair._obj_id2];

		const auto shape1 = obj1->_collision_shape;
		const auto shape2 = obj2->_collision_shape;

        const auto& vertices1 = shape1->GetCollisionVertices();
        const auto& vertices2 = shape2->GetCollisionVertices();
        
        const int offset1 = _offsets[constraint_pair._obj_id1];
        const int offset2 = _offsets[constraint_pair._obj_id2];

        switch (constraint_pair._type) {
            case CollisionType::kVertexFace: {
                const int vertex_index = constraint_pair._primitive_id1;
                const RowVector3i face_index = shape2->GetCollisionFaceTopo().row(constraint_pair._primitive_id2);
                const Vector3d
                    vertex = vertices1.row(vertex_index),
                    face1 = vertices2.row(face_index(0)),
                    face2 = vertices2.row(face_index(1)),
                    face3 = vertices2.row(face_index(2));
                
                Matrix12d single_hessian = _kappa * GetVFBarrierEnergyHessian(vertex, face1, face2, face3);
                
                const int offset[4] = {offset1, offset2, offset2, offset2};
				const int index[4] = {vertex_index, face_index(0), face_index(1), face_index(2)};
                const CollisionShape* shape[4] = {shape1, shape2, shape2, shape2};

				for (int i = 0, ii = 0; i < 4; i++, ii += 3) {
					for (int j = 0, jj = 0; j < 4; j++, jj += 3) {
                        shape[i]->GetVertexDerivative(index[i])
                        .RightProduct(single_hessian.block<3, 3>(ii, jj))
                        .RightTransposeProduct(shape[j]->GetVertexDerivative(index[j]))
                        .ToSparse(coo, offset[i] + offset_x, offset[j] + offset_y);
					}
				}

                break;
            }
            case CollisionType::kEdgeEdge: {
                const RowVector2i
                    edge_index1 = shape1->GetCollisionEdgeTopo().row(constraint_pair._primitive_id1),
                    edge_index2 = shape2->GetCollisionEdgeTopo().row(constraint_pair._primitive_id2);
                const Vector3d
                    edge11 = vertices1.row(edge_index1(0)),
                    edge12 = vertices1.row(edge_index1(1)),
                    edge21 = vertices2.row(edge_index2(0)),
                    edge22 = vertices2.row(edge_index2(1));
                
                Matrix12d single_hessian = _kappa * GetEEBarrierEnergyHessian(edge11, edge12, edge21, edge22);

                const int offset[4] = {offset1, offset1, offset2, offset2};
				const int index[4] = {edge_index1(0), edge_index1(1), edge_index2(0), edge_index2(1)};
                const CollisionShape* shape[4] = {shape1, shape1, shape2, shape2};

				for (int i = 0, ii = 0; i < 4; i++, ii += 3) {
					for (int j = 0, jj = 0; j < 4; j++, jj += 3) {
                        shape[i]->GetVertexDerivative(index[i])
                        .RightProduct(single_hessian.block<3, 3>(ii, jj))
                        .RightTransposeProduct(shape[j]->GetVertexDerivative(index[j]))
                        .ToSparse(coo, offset[i] + offset_x, offset[j] + offset_y);
					}
				}
                break;
            }
        }
    }
}

double IPCBarrierTarget::GetMaxStep(const Eigen::VectorXd &p) {
    double max_step_inside = 1;

	PROCESS_CONSTRAINT_PAIR(
		const int offset1 = _offsets[constraint_pair._obj_id1];
		const int dof1 = obj1->GetDOF();
		const auto& p1 = p.segment(offset1, dof1);
		const int offset2 = _offsets[constraint_pair._obj_id2];
		const int dof2 = obj2->GetDOF();
		const auto& p2 = p.segment(offset2, dof2);,

		max_step_inside = std::min(
			max_step_inside,
			_ccd->VertexFaceCollision(
				rotation1 * vertex + translation1,
				rotation2 * face1 + translation2,
				rotation2 * face2 + translation2,
				rotation2 * face3 + translation2,
				rotation1 * shape1->GetCollisionVertexVelocity(p1, vertex_index),
				rotation2 * shape2->GetCollisionVertexVelocity(p2, face_index(0)),
				rotation2 * shape2->GetCollisionVertexVelocity(p2, face_index(1)),
				rotation2 * shape2->GetCollisionVertexVelocity(p2, face_index(2))
			)
		);,

		max_step_inside = std::min(
			max_step_inside,
			_ccd->EdgeEdgeCollision(
				rotation1 * edge11 + translation1,
				rotation1 * edge12 + translation1,
				rotation2 * edge21 + translation2,
				rotation2 * edge22 + translation2,
				rotation1 * shape1->GetCollisionVertexVelocity(p1, edge_index1(0)),
				rotation1 * shape1->GetCollisionVertexVelocity(p1, edge_index1(1)),
				rotation2 * shape2->GetCollisionVertexVelocity(p2, edge_index2(0)),
				rotation2 * shape2->GetCollisionVertexVelocity(p2, edge_index2(1))
			)
		);
	)

	double max_velocity = 0;
	int cur_offset = 0;
	for (const auto obj : _objs) {
		max_velocity = std::max(max_velocity, obj->GetMaxVelocity(p.segment(cur_offset, obj->GetDOF())));
		cur_offset += obj->GetDOF();
	}

	double max_step_outside = max_velocity > 0 ? _d_hat / (2 * max_velocity) : 1;

	double max_step;
	if (max_step_outside > 0.5 * max_step_inside) {
		max_step = std::min(max_step_inside, max_step_outside);
	} else {
		max_step = GetFullCCD(p);
	}

	// spdlog::info("inside step: {}, outside step: {}, final step: {}", max_step_inside, max_step_outside, max_step);

	if (max_step >= 1) {
		return max_step;
	} else {
		return max_step * 0.8;
	}

	// if (max_step < 1) {
	// 	spdlog::info("Max step for newton: {}", max_step);
	// 	exit(EXIT_FAILURE);
	// }
}

void IPCBarrierTarget::ComputeConstraintSet(const Eigen::VectorXd &x) {
	    /* insert into hash table */
	_constraint_set.clear();

    int cur_offset = 0;
	for (auto obj : _objs) {
		obj->_collision_shape->ComputeCollisionShape(x.segment(cur_offset, obj->GetDOF()));
		cur_offset += obj->GetDOF();
	}

	_time_stamp++;

	cur_offset = 0;
    int obj_id = 0;
    for (auto obj : _objs) {
        const Matrix3d rotation = obj->GetFrameRotation();
        const Vector3d translation = obj->GetFrameX();

		const auto& vertices = obj->_collision_shape->GetCollisionVertices();
		const auto& face_topo = obj->_collision_shape->GetCollisionFaceTopo();
		const auto& edge_topo = obj->_collision_shape->GetCollisionEdgeTopo();
        const int num_vertices = vertices.rows();
        const int num_faces = face_topo.rows();
        const int num_edges = edge_topo.rows();

        for (int i = 0; i < num_vertices; i++) {
            Vector3d vertice = rotation * vertices.row(i).transpose() + translation;
            _vertex_hash_table.Insert(
                vertice,
                VertexPrimitiveInfo {obj_id, i, vertice},
                _time_stamp
            );
        }

        for (int i = 0; i < num_edges; i++) {
            Vector3d edge_vertex1 = rotation * vertices.row(edge_topo(i, 0)).transpose() + translation;
            Vector3d edge_vertex2 = rotation * vertices.row(edge_topo(i, 1)).transpose() + translation;
            Vector3d bb_min = edge_vertex1.cwiseMin(edge_vertex2);
            Vector3d bb_max = edge_vertex1.cwiseMax(edge_vertex2);

            _edge_hash_table.Insert(
                bb_min, bb_max,
                EdgePrimitiveInfo {obj_id, i, edge_vertex1, edge_vertex2},
                _time_stamp
            );
        }
        cur_offset += obj->GetDOF();
        obj_id++;
    }

    /* find in hash table */

    obj_id = 0;
    for (auto obj : _objs) {
        const Matrix3d rotation = obj->GetFrameRotation();
        const Vector3d translation = obj->GetFrameX();

		const auto& vertices = obj->_collision_shape->GetCollisionVertices();
		const auto& face_topo = obj->_collision_shape->GetCollisionFaceTopo();
		const auto& edge_topo = obj->_collision_shape->GetCollisionEdgeTopo();

        const int num_faces = face_topo.rows();
        const int num_edges = edge_topo.rows();

        /* vertex-face collision */
        for (int i = 0; i < num_faces; i++) {
            const auto topo = face_topo.row(i);
            Vector3d face_vertex1 = rotation * vertices.row(topo(0)).transpose() + translation;
            Vector3d face_vertex2 = rotation * vertices.row(topo(1)).transpose() + translation;
            Vector3d face_vertex3 = rotation * vertices.row(topo(2)).transpose() + translation;

            Vector3d bb_min = face_vertex1.cwiseMin(face_vertex2.cwiseMin(face_vertex3)) - Vector3d::Constant(_d_hat);
            Vector3d bb_max = face_vertex1.cwiseMax(face_vertex2.cwiseMax(face_vertex3)) + Vector3d::Constant(_d_hat);

            auto candidates = _vertex_hash_table.Find(bb_min, bb_max, _time_stamp);
            for (const auto& candidate : candidates) {
				if (candidate._obj_id == obj_id && (candidate._primitive_id == topo(0) || candidate._primitive_id == topo(1) || candidate._primitive_id == topo(2))) {
					continue;
				}
                if (GetVFDistance(candidate._vertex, face_vertex1, face_vertex2, face_vertex3) < _d_hat) {
                    _constraint_set.push_back(CollisionInfo{
                        CollisionType::kVertexFace,
                        candidate._obj_id, obj_id,
                        candidate._primitive_id, i
                    });
                }
            }
        }

        /* edge edge collision */
        for (int i = 0; i < num_edges; i++) {
            const auto topo = edge_topo.row(i);

            Vector3d edge_vertex1 = rotation * vertices.row(topo(0)).transpose() + translation;
            Vector3d edge_vertex2 = rotation * vertices.row(topo(1)).transpose() + translation;

            Vector3d bb_min = edge_vertex1.cwiseMin(edge_vertex2) - Vector3d::Constant(_d_hat);
            Vector3d bb_max = edge_vertex1.cwiseMax(edge_vertex2) + Vector3d::Constant(_d_hat);

            auto candidates = _edge_hash_table.Find(bb_min, bb_max, _time_stamp);
            for (const auto& candidate : candidates) {
				if (candidate._obj_id > obj_id) {
					continue;
				}
				if (candidate._obj_id == obj_id) {
					RowVector2i other_topo = _objs[candidate._obj_id]->_collision_shape->GetCollisionEdgeTopo().row(candidate._primitive_id);
					if (other_topo(0) == topo(0) || other_topo(0) == topo(1) || other_topo(1) == topo(0) || other_topo(1) == topo(1)) {
						continue;
					}
				}
                if (GetEEDistance(candidate._vertex1, candidate._vertex2, edge_vertex1, edge_vertex2) < _d_hat) {
                    _constraint_set.push_back(CollisionInfo{
                        CollisionType::kEdgeEdge,
                        candidate._obj_id, obj_id,
                        candidate._primitive_id, i
                    });
                }
            }
        }
		obj_id++;
    }

    // if (!_constraint_set.empty()) {
    //     spdlog::info("Constraint set size: {}", _constraint_set.size());
    // }

	// if (!_constraint_set.empty()) {
	// 	for (const auto& constraint_pair : _constraint_set) {
	// 		std::cerr << (constraint_pair._type == CollisionType::kEdgeEdge ? "edge-edge" : "vertex-face") << std::endl
	// 				  << "Objects: " << constraint_pair._obj_id1 << " " << constraint_pair._obj_id2 << std::endl
	// 				  << "Primitives: " << constraint_pair._primitive_id1 << " " << constraint_pair._primitive_id2 << std::endl;
	// 	}
	// 	exit(-1);
	// }
}

double IPCBarrierTarget::GetFullCCD(const VectorXd& p) {
	// FULL CCD using hashing
	_time_stamp++;

	int cur_offset = 0;
    int obj_id = 0;
    for (auto obj : _objs) {
        const Matrix3d rotation = obj->GetFrameRotation();
        const Vector3d translation = obj->GetFrameX();

		const auto& vertices = obj->_collision_shape->GetCollisionVertices();
		const auto& face_topo = obj->_collision_shape->GetCollisionFaceTopo();
		const auto& edge_topo = obj->_collision_shape->GetCollisionEdgeTopo();
		const auto& p_obj = p.segment(cur_offset, obj->GetDOF());
        const int num_vertices = vertices.rows();
        const int num_faces = face_topo.rows();
        const int num_edges = edge_topo.rows();

        for (int i = 0; i < num_vertices; i++) {
            Vector3d vertice = rotation * vertices.row(i).transpose() + translation;
			Vector3d vertice_next = rotation * (vertices.row(i).transpose() + obj->_collision_shape->GetCollisionVertexVelocity(p_obj, i)) + translation;

			Vector3d min_point = vertice.cwiseMin(vertice_next);
			Vector3d max_point = vertice.cwiseMax(vertice_next);

            _vertex_hash_table.Insert(
                min_point, max_point,
                VertexPrimitiveInfo {obj_id, i, vertice},
                _time_stamp
            );
        }

        for (int i = 0; i < num_edges; i++) {
            Vector3d edge_vertex1 = rotation * vertices.row(edge_topo(i, 0)).transpose() + translation;
			Vector3d edge_vertex_next1 = rotation * (vertices.row(edge_topo(i, 0)).transpose() + obj->_collision_shape->GetCollisionVertexVelocity(p_obj, edge_topo(i, 0))) + translation;
            Vector3d edge_vertex2 = rotation * vertices.row(edge_topo(i, 1)).transpose() + translation;
			Vector3d edge_vertex_next2 = rotation * (vertices.row(edge_topo(i, 1)).transpose() + obj->_collision_shape->GetCollisionVertexVelocity(p_obj, edge_topo(i, 1))) + translation;
            Vector3d bb_min = edge_vertex1.cwiseMin(edge_vertex_next1).cwiseMin(edge_vertex2.cwiseMin(edge_vertex_next2));
            Vector3d bb_max = edge_vertex1.cwiseMax(edge_vertex_next1).cwiseMax(edge_vertex2.cwiseMax(edge_vertex_next2));

            _edge_hash_table.Insert(
                bb_min, bb_max,
                EdgePrimitiveInfo {obj_id, i, edge_vertex1, edge_vertex2},
                _time_stamp
            );
        }
        cur_offset += obj->GetDOF();
        obj_id++;
    }

    /* find in hash table */

	cur_offset = 0;
    obj_id = 0;

	double ttc = 1;

    for (auto obj : _objs) {
        const Matrix3d rotation = obj->GetFrameRotation();
        const Vector3d translation = obj->GetFrameX();

		const auto& vertices = obj->_collision_shape->GetCollisionVertices();
		const auto& face_topo = obj->_collision_shape->GetCollisionFaceTopo();
		const auto& edge_topo = obj->_collision_shape->GetCollisionEdgeTopo();
		const auto& p_obj = p.segment(cur_offset, obj->GetDOF());

        const int num_faces = face_topo.rows();
        const int num_edges = edge_topo.rows();

        /* vertex-face collision */
        for (int i = 0; i < num_faces; i++) {
            const auto topo = face_topo.row(i);
            Vector3d face_vertex1 = rotation * vertices.row(topo(0)).transpose() + translation;
			Vector3d face_vertex_v1 = rotation * obj->_collision_shape->GetCollisionVertexVelocity(p_obj, topo(0));
            Vector3d face_vertex_next1 = face_vertex1 + face_vertex_v1;
            Vector3d face_vertex2 = rotation * vertices.row(topo(1)).transpose() + translation;
			Vector3d face_vertex_v2 = rotation * obj->_collision_shape->GetCollisionVertexVelocity(p_obj, topo(1));
            Vector3d face_vertex_next2 = face_vertex2 + face_vertex_v2;
            Vector3d face_vertex3 = rotation * vertices.row(topo(2)).transpose() + translation;
			Vector3d face_vertex_v3 = rotation * obj->_collision_shape->GetCollisionVertexVelocity(p_obj, topo(2));
            Vector3d face_vertex_next3 = face_vertex3 + face_vertex_v3;

            Vector3d bb_min = face_vertex1.cwiseMin(face_vertex2.cwiseMin(face_vertex3)).cwiseMin(face_vertex_next1.cwiseMin(face_vertex_next2.cwiseMin(face_vertex_next3)));
            Vector3d bb_max = face_vertex1.cwiseMax(face_vertex2.cwiseMax(face_vertex3)).cwiseMax(face_vertex_next1.cwiseMax(face_vertex_next2.cwiseMax(face_vertex_next3)));

            auto candidates = _vertex_hash_table.Find(bb_min, bb_max, _time_stamp);
            for (const auto& candidate : candidates) {
				if (candidate._obj_id == obj_id && (candidate._primitive_id == topo(0) || candidate._primitive_id == topo(1) || candidate._primitive_id == topo(2))) {
					continue;
				}
				const auto other_obj = _objs[candidate._obj_id];
				const Vector3d vertex = other_obj->GetFrameRotation() * other_obj->_collision_shape->GetCollisionVertices().row(candidate._primitive_id).transpose() + other_obj->GetFrameX();
				const Vector3d vertex_v = other_obj->GetFrameRotation() * other_obj->_collision_shape->GetCollisionVertexVelocity(p.segment(_offsets[candidate._obj_id], other_obj->GetDOF()), candidate._primitive_id);

				double new_ttc =
					_ccd->VertexFaceCollision(
						vertex, face_vertex1, face_vertex2, face_vertex3,
						vertex_v, face_vertex_v1, face_vertex_v2, face_vertex_v3
					);

				ttc = std::min(
					ttc,
					new_ttc
				);
				if (new_ttc < 1e-3) {
					spdlog::info("Vertex-Face case: Object1 = {}, Primitive1 = {}, Object2 = {}, Primitive2 = {}", obj_id, i, candidate._obj_id, candidate._primitive_id);

					std::cerr << "Vertex: " << vertex.transpose() << std::endl;
					std::cerr << "Face: " << face_vertex1.transpose() << ", " << face_vertex2.transpose() << ", " << face_vertex3.transpose() << std::endl;
				}
            }
        }

        /* edge edge collision */
        for (int i = 0; i < num_edges; i++) {
            const auto topo = edge_topo.row(i);

            Vector3d edge_vertex1 = rotation * vertices.row(topo(0)).transpose() + translation;
			Vector3d edge_vertex_v1 = rotation * obj->_collision_shape->GetCollisionVertexVelocity(p_obj, topo(0));
			Vector3d edge_vertex_next1 = edge_vertex1 + edge_vertex_v1;
            Vector3d edge_vertex2 = rotation * vertices.row(topo(1)).transpose() + translation;
			Vector3d edge_vertex_v2 = rotation * obj->_collision_shape->GetCollisionVertexVelocity(p_obj, topo(1));
			Vector3d edge_vertex_next2 = edge_vertex2 + edge_vertex_v2;
			
            Vector3d bb_min = edge_vertex1.cwiseMin(edge_vertex_next1).cwiseMin(edge_vertex2.cwiseMin(edge_vertex_next2));
            Vector3d bb_max = edge_vertex1.cwiseMax(edge_vertex_next1).cwiseMax(edge_vertex2.cwiseMax(edge_vertex_next2));

            auto candidates = _edge_hash_table.Find(bb_min, bb_max, _time_stamp);
            for (const auto& candidate : candidates) {
				if (candidate._obj_id > obj_id) {
					continue;
				}
				if (candidate._obj_id == obj_id) {
					RowVector2i other_topo = _objs[candidate._obj_id]->_collision_shape->GetCollisionEdgeTopo().row(candidate._primitive_id);
					if (other_topo(0) == topo(0) || other_topo(0) == topo(1) || other_topo(1) == topo(0) || other_topo(1) == topo(1)) {
						continue;
					}
				}
				const auto other_obj = _objs[candidate._obj_id];
				const auto other_p_obj = p.segment(_offsets[candidate._obj_id], other_obj->GetDOF());
				const auto other_topo = other_obj->_collision_shape->GetCollisionEdgeTopo().row(candidate._primitive_id);
				Vector3d other_edge_vertex1 = other_obj->GetFrameRotation() * other_obj->_collision_shape->GetCollisionVertices().row(other_topo(0)).transpose() + other_obj->GetFrameX();
				Vector3d other_edge_vertex_v1 = other_obj->GetFrameRotation() * other_obj->_collision_shape->GetCollisionVertexVelocity(other_p_obj, other_topo(0));
				Vector3d other_edge_vertex2 = other_obj->GetFrameRotation() * other_obj->_collision_shape->GetCollisionVertices().row(other_topo(1)).transpose() + other_obj->GetFrameX();
				Vector3d other_edge_vertex_v2 = other_obj->GetFrameRotation() * other_obj->_collision_shape->GetCollisionVertexVelocity(other_p_obj, other_topo(1));
				
				double new_ttc = 
					_ccd->EdgeEdgeCollision(
						edge_vertex1, edge_vertex2, other_edge_vertex1, other_edge_vertex2,
						edge_vertex_v1, edge_vertex_v2, other_edge_vertex_v1, other_edge_vertex_v2
					);

				ttc = std::min (
					ttc,
					new_ttc
				);

				if (new_ttc < 1e-5) {
					spdlog::info("Edge-edge case: Object1 = {}, Primitive1 = {}, Object2 = {}, Primitive2 = {}", obj_id, i, candidate._obj_id, candidate._primitive_id);
				}
            }
        }
		cur_offset += obj->GetDOF();
		obj_id++;
    }

	// spdlog::info("Full ccd, ttc = {}", ttc);

	return ttc;
}

double IPCBarrierTarget::GetVFBarrierEnergy(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const {
    double d = GetVFDistance(vertex, face1, face2, face3);
    return -(d - _d_hat) * (d - _d_hat) * log(d / _d_hat);
}

Vector12d IPCBarrierTarget::GetVFBarrierEnergyGradient(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const {
    double d = GetVFDistance(vertex, face1, face2, face3);
    Vector12d pdpx = GetVFDistanceGradient(vertex, face1, face2, face3);

    double pbpd = -2 * (d - _d_hat) * log(d / _d_hat) - (d - _d_hat) * (d - _d_hat) / d;
    return  pbpd * pdpx;
}

Matrix12d IPCBarrierTarget::GetVFBarrierEnergyHessian(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const {
    double d = GetVFDistance(vertex, face1, face2, face3);
    Vector12d pdpx = GetVFDistanceGradient(vertex, face1, face2, face3);
    Matrix12d p2dpx2 = GetVFDistanceHessian(vertex, face1, face2, face3);
    
    const double pbpd = -2 * (d - _d_hat) * log(d / _d_hat) - (d - _d_hat) * (d - _d_hat) / d;
    const double p2bpd2 = -2 * log(d / _d_hat) - 4 * (d - _d_hat) / d + (d - _d_hat) * (d - _d_hat) / (d * d);
    return PositiveProject<12>(p2bpd2 * pdpx * pdpx.transpose() + pbpd * p2dpx2);
}

double IPCBarrierTarget::GetEEBarrierEnergy(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const {
    double d = GetEEDistance(edge11, edge12, edge21, edge22);
    return -(d - _d_hat) * (d - _d_hat) * log(d / _d_hat);
}

Vector12d IPCBarrierTarget::GetEEBarrierEnergyGradient(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const {
    double d = GetEEDistance(edge11, edge12, edge21, edge22);
    Vector12d pdpx = GetEEDistanceGradient(edge11, edge12, edge21, edge22);

    double pbpd = -2 * (d - _d_hat) * log(d / _d_hat) - (d - _d_hat) * (d - _d_hat) / d;
    return  pbpd * pdpx;    
}

Matrix12d IPCBarrierTarget::GetEEBarrierEnergyHessian(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const {
    double d = GetEEDistance(edge11, edge12, edge21, edge22);
    Vector12d pdpx = GetEEDistanceGradient(edge11, edge12, edge21, edge22);
    Matrix12d p2dpx2 = GetEEDistanceHessian(edge11, edge12, edge21, edge22);
    
    const double pbpd = -2 * (d - _d_hat) * log(d / _d_hat) - (d - _d_hat) * (d - _d_hat) / d;
    const double p2bpd2 = -2 * log(d / _d_hat) - 4 * (d - _d_hat) / d + (d - _d_hat) * (d - _d_hat) / (d * d);
    return PositiveProject<12>(p2bpd2 * pdpx * pdpx.transpose() + pbpd * p2dpx2);
}

IPCBarrierTarget::~IPCBarrierTarget() {
    delete _ccd;
}
