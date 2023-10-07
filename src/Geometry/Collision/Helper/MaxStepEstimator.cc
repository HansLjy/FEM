#include "MaxStepEstimator.hpp"

#define PROCESS_CCD_SET(Initialization, VF, EE) \
    for (const auto& primitive_pair : _ccd_set) {\
		const int id1 = primitive_pair._obj_id1;\
		const int id2 = primitive_pair._obj_id2;\
        const auto obj1 = objs[primitive_pair._obj_id1];\
        const auto obj2 = objs[primitive_pair._obj_id2];\
		\
        const auto& vertices1 = obj1->GetCollisionVertices();\
        const auto& vertices2 = obj2->GetCollisionVertices();\
        \
        Initialization \
        switch (primitive_pair._type) {\
            case CollisionType::kVertexFace: {\
                const int vertex_index = primitive_pair._primitive_id1;\
                const RowVector3i face_index = obj2->GetCollisionFaceTopo().row(primitive_pair._primitive_id2);\
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
                    edge_index1 = obj1->GetCollisionEdgeTopo().row(primitive_pair._primitive_id1),\
                    edge_index2 = obj2->GetCollisionEdgeTopo().row(primitive_pair._primitive_id2);\
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

double NormalMaxStepEstimator::GetMaxStep(
	const VectorXd &p,
	double d_hat,
	const std::vector<CollisionShapeInterface *> &objs,
	const std::vector<int> &offsets,
	const std::vector<int> &dofs) {
	double max_step = 1;

	int obj_id = 0;
	for (auto obj : objs) {
		obj->ComputeCollisionVertexVelocity(p.segment(offsets[obj_id], dofs[obj_id]));
		obj_id++;
	}

	_culling->GetCCDSet(d_hat, objs, offsets, dofs, _ccd_set);

	PROCESS_CCD_SET(
		const int offset1 = offsets[primitive_pair._obj_id1];
		const int dof1 = dofs[id1];
		const auto& p1 = p.segment(offset1, dof1);
		const int offset2 = offsets[primitive_pair._obj_id2];
		const int dof2 = dofs[id2];
		const auto& p2 = p.segment(offset2, dof2);,

		max_step = std::min(
			max_step,
			_ccd->VertexFaceCollision(
				vertex, face1, face2, face3,
				obj1->GetCollisionVertexVelocity(vertex_index),
				obj2->GetCollisionVertexVelocity(face_index(0)),
				obj2->GetCollisionVertexVelocity(face_index(1)),
				obj2->GetCollisionVertexVelocity(face_index(2))
			)
		);,

		max_step = std::min(
			max_step,
			_ccd->EdgeEdgeCollision(
				edge11, edge12, edge21, edge22,
				obj1->GetCollisionVertexVelocity(edge_index1(0)),
				obj1->GetCollisionVertexVelocity(edge_index1(1)),
				obj2->GetCollisionVertexVelocity(edge_index2(0)),
				obj2->GetCollisionVertexVelocity(edge_index2(1))
			)
		);
	)

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