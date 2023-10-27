//
// Created by hansljy on 12/2/22.
//

#include "IPCHelper.hpp"
#include "Collision/CollisionUtility.h"
#include "spdlog/spdlog.h"
#include "Collision/CollisionShape/CollisionShape.hpp"

void IPCEnergy::BindObjects(
    const typename std::vector<Object>::const_iterator &begin,
    const typename std::vector<Object>::const_iterator &end
) {
    InterfaceContainer::BindObjects(begin, end);
    _total_dof = 0;
    _offsets.clear();
    _offsets.reserve(end - begin);
    for (const auto& obj : _objs) {
        _offsets.push_back(_total_dof);
        _total_dof += obj.GetDOF();
    }
}

#define PROCESS_PRIMITIVE_PAIRS(PrimitiveSet, Initialization, VF, EE) \
    for (const auto& primitive_pair : PrimitiveSet) {\
		const int id1 = primitive_pair._obj_id1;\
		const int id2 = primitive_pair._obj_id2;\
        const auto obj1 = _objs[primitive_pair._obj_id1];\
        const auto obj2 = _objs[primitive_pair._obj_id2];\
		\
        const auto& vertices1 = obj1.GetCollisionVertices();\
        const auto& vertices2 = obj2.GetCollisionVertices();\
        \
        Initialization \
        switch (primitive_pair._type) {\
            case CollisionType::kVertexFace: {\
                const int vertex_index = primitive_pair._primitive_id1;\
                const RowVector3i face_index = obj2.GetCollisionFaceTopo().row(primitive_pair._primitive_id2);\
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
                    edge_index1 = obj1.GetCollisionEdgeTopo().row(primitive_pair._primitive_id1),\
                    edge_index2 = obj2.GetCollisionEdgeTopo().row(primitive_pair._primitive_id2);\
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

double IPCEnergy::GetBarrierEnergy(const std::vector<PrimitivePair>& constraint_set) const {
    double barrier_energy = 0;
    PROCESS_PRIMITIVE_PAIRS(
        constraint_set,
        ,
        barrier_energy += GetVFBarrierEnergy(vertex, face1, face2, face3);,
        barrier_energy += GetEEBarrierEnergy(edge11, edge12, edge21, edge22);
    )
    return barrier_energy * _kappa;
}

VectorXd IPCEnergy::GetBarrierEnergyGradient(const std::vector<PrimitivePair> &constraint_set) const {
    VectorXd gradient(_total_dof);
    gradient.setZero();

    PROCESS_PRIMITIVE_PAIRS (
        constraint_set,
        const int dof1 = _objs[id1].GetDOF();
        const int dof2 = _objs[id2].GetDOF();
        const int offset1 = _offsets[primitive_pair._obj_id1];
        const int offset2 = _offsets[primitive_pair._obj_id2];,

        const Vector12d single_gradient = GetVFBarrierEnergyGradient(vertex, face1, face2, face3);
        obj1.GetCollisionVertexDerivative(vertex_index).RightProduct(single_gradient.segment<3>(0), gradient.segment(offset1, dof1));
        obj2.GetCollisionVertexDerivative(face_index(0)).RightProduct(single_gradient.segment<3>(3), gradient.segment(offset2, dof2));
        obj2.GetCollisionVertexDerivative(face_index(1)).RightProduct(single_gradient.segment<3>(6), gradient.segment(offset2, dof2));
        obj2.GetCollisionVertexDerivative(face_index(2)).RightProduct(single_gradient.segment<3>(9), gradient.segment(offset2, dof2));
        ,

        const Vector12d single_gradient = GetEEBarrierEnergyGradient(edge11, edge12, edge21, edge22);
        obj1.GetCollisionVertexDerivative(edge_index1(0)).RightProduct(single_gradient.segment<3>(0), gradient.segment(offset1, dof1));
        obj1.GetCollisionVertexDerivative(edge_index1(1)).RightProduct(single_gradient.segment<3>(3), gradient.segment(offset1, dof1));
        obj2.GetCollisionVertexDerivative(edge_index2(0)).RightProduct(single_gradient.segment<3>(6), gradient.segment(offset2, dof2));
        obj2.GetCollisionVertexDerivative(edge_index2(1)).RightProduct(single_gradient.segment<3>(9), gradient.segment(offset2, dof2));
    )

    return gradient * _kappa;
}

void IPCEnergy::GetBarrierEnergyHessian(
    const std::vector<PrimitivePair> &constraint_set,
    COO &coo, int offset_x, int offset_y
) const {
    for (const auto& constraint_pair : constraint_set) {
        const auto obj1 = _objs[constraint_pair._obj_id1];
        const auto obj2 = _objs[constraint_pair._obj_id2];

        const auto& vertices1 = obj1.GetCollisionVertices();
        const auto& vertices2 = obj2.GetCollisionVertices();
        
        const int offset1 = _offsets[constraint_pair._obj_id1];
        const int offset2 = _offsets[constraint_pair._obj_id2];

        switch (constraint_pair._type) {
            case CollisionType::kVertexFace: {
                const int vertex_index = constraint_pair._primitive_id1;
                const RowVector3i face_index = obj2.GetCollisionFaceTopo().row(constraint_pair._primitive_id2);
                const Vector3d
                    vertex = vertices1.row(vertex_index),
                    face1 = vertices2.row(face_index(0)),
                    face2 = vertices2.row(face_index(1)),
                    face3 = vertices2.row(face_index(2));
                
                Matrix12d single_hessian = _kappa * GetVFBarrierEnergyHessian(vertex, face1, face2, face3);
                
                const int offset[4] = {offset1, offset2, offset2, offset2};
				const int index[4] = {vertex_index, face_index(0), face_index(1), face_index(2)};
                const CollisionInterface* shape[4] = {&obj1, &obj2, &obj2, &obj2};

				for (int i = 0, ii = 0; i < 4; i++, ii += 3) {
					for (int j = 0, jj = 0; j < 4; j++, jj += 3) {
                        shape[i]->GetCollisionVertexDerivative(index[i])
                        .RightProduct(single_hessian.block<3, 3>(ii, jj))
                        .RightTransposeProduct(shape[j]->GetCollisionVertexDerivative(index[j]))
                        .ToSparse(coo, offset[i] + offset_x, offset[j] + offset_y);
					}
				}

                break;
            }
            case CollisionType::kEdgeEdge: {
                const RowVector2i
                    edge_index1 = obj1.GetCollisionEdgeTopo().row(constraint_pair._primitive_id1),
                    edge_index2 = obj2.GetCollisionEdgeTopo().row(constraint_pair._primitive_id2);
                const Vector3d
                    edge11 = vertices1.row(edge_index1(0)),
                    edge12 = vertices1.row(edge_index1(1)),
                    edge21 = vertices2.row(edge_index2(0)),
                    edge22 = vertices2.row(edge_index2(1));
                
                Matrix12d single_hessian = _kappa * GetEEBarrierEnergyHessian(edge11, edge12, edge21, edge22);

                const int offset[4] = {offset1, offset1, offset2, offset2};
				const int index[4] = {edge_index1(0), edge_index1(1), edge_index2(0), edge_index2(1)};
                const CollisionInterface* shape[4] = {&obj1, &obj1, &obj2, &obj2};

				for (int i = 0, ii = 0; i < 4; i++, ii += 3) {
					for (int j = 0, jj = 0; j < 4; j++, jj += 3) {
                        shape[i]->GetCollisionVertexDerivative(index[i])
                        .RightProduct(single_hessian.block<3, 3>(ii, jj))
                        .RightTransposeProduct(shape[j]->GetCollisionVertexDerivative(index[j]))
                        .ToSparse(coo, offset[i] + offset_x, offset[j] + offset_y);
					}
				}
                break;
            }
        }
    }
}

double IPCEnergy::GetVFBarrierEnergy(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const {
    double d = GetVFDistance(vertex, face1, face2, face3);
    return -(d - _d_hat) * (d - _d_hat) * log(d / _d_hat);
}

Vector12d IPCEnergy::GetVFBarrierEnergyGradient(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const {
    double d = GetVFDistance(vertex, face1, face2, face3);
    Vector12d pdpx = GetVFDistanceGradient(vertex, face1, face2, face3);

    double pbpd = -2 * (d - _d_hat) * log(d / _d_hat) - (d - _d_hat) * (d - _d_hat) / d;
    return  pbpd * pdpx;
}

Matrix12d IPCEnergy::GetVFBarrierEnergyHessian(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const {
    double d = GetVFDistance(vertex, face1, face2, face3);
    Vector12d pdpx = GetVFDistanceGradient(vertex, face1, face2, face3);
    Matrix12d p2dpx2 = GetVFDistanceHessian(vertex, face1, face2, face3);
    
    const double pbpd = -2 * (d - _d_hat) * log(d / _d_hat) - (d - _d_hat) * (d - _d_hat) / d;
    const double p2bpd2 = -2 * log(d / _d_hat) - 4 * (d - _d_hat) / d + (d - _d_hat) * (d - _d_hat) / (d * d);
    return PositiveProject<12>(p2bpd2 * pdpx * pdpx.transpose() + pbpd * p2dpx2);
}

double IPCEnergy::GetEEBarrierEnergy(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const {
    double d = GetEEDistance(edge11, edge12, edge21, edge22);
    return -(d - _d_hat) * (d - _d_hat) * log(d / _d_hat);
}

Vector12d IPCEnergy::GetEEBarrierEnergyGradient(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const {
    double d = GetEEDistance(edge11, edge12, edge21, edge22);
    Vector12d pdpx = GetEEDistanceGradient(edge11, edge12, edge21, edge22);

    double pbpd = -2 * (d - _d_hat) * log(d / _d_hat) - (d - _d_hat) * (d - _d_hat) / d;
    return  pbpd * pdpx;    
}

Matrix12d IPCEnergy::GetEEBarrierEnergyHessian(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const {
    double d = GetEEDistance(edge11, edge12, edge21, edge22);
    Vector12d pdpx = GetEEDistanceGradient(edge11, edge12, edge21, edge22);
    Matrix12d p2dpx2 = GetEEDistanceHessian(edge11, edge12, edge21, edge22);
    
    const double pbpd = -2 * (d - _d_hat) * log(d / _d_hat) - (d - _d_hat) * (d - _d_hat) / d;
    const double p2bpd2 = -2 * log(d / _d_hat) - 4 * (d - _d_hat) / d + (d - _d_hat) * (d - _d_hat) / (d * d);
    return PositiveProject<12>(p2bpd2 * pdpx * pdpx.transpose() + pbpd * p2dpx2);
}

void MaxStepEstimator::BindObjects(
    const typename std::vector<Object>::const_iterator &begin,
    const typename std::vector<Object>::const_iterator &end
) {
    InterfaceContainer::BindObjects(begin, end);
    int total_dof = 0;
    _offsets.clear();
    _offsets.reserve(end - begin);
    for (const auto& obj : _objs) {
        _offsets.push_back(total_dof);
        total_dof += obj.GetDOF();
    }
}

double MaxStepEstimator::GetMaxStep(const VectorXd &p) {
    double max_step = 1;

	int obj_id = 0;
	for (auto obj : _objs) {
		obj.ComputeCollisionVertexVelocity(p.segment(_offsets[obj_id], _objs[obj_id].GetDOF()));
		obj_id++;
	}

	_culling->GetCCDSet(_objs, _offsets, _ccd_set);

	PROCESS_PRIMITIVE_PAIRS(
        _ccd_set,
		const int offset1 = _offsets[primitive_pair._obj_id1];
		const int dof1 = _objs[id1].GetDOF();
		const auto& p1 = p.segment(offset1, dof1);
		const int offset2 = _offsets[primitive_pair._obj_id2];
		const int dof2 = _objs[id2].GetDOF();
		const auto& p2 = p.segment(offset2, dof2);,

		max_step = std::min(
			max_step,
			_ccd->VertexFaceCollision(
				vertex, face1, face2, face3,
				obj1.GetCollisionVertexVelocity(vertex_index),
				obj2.GetCollisionVertexVelocity(face_index(0)),
				obj2.GetCollisionVertexVelocity(face_index(1)),
				obj2.GetCollisionVertexVelocity(face_index(2))
			)
		);,

		max_step = std::min(
			max_step,
			_ccd->EdgeEdgeCollision(
				edge11, edge12, edge21, edge22,
				obj1.GetCollisionVertexVelocity(edge_index1(0)),
				obj1.GetCollisionVertexVelocity(edge_index1(1)),
				obj2.GetCollisionVertexVelocity(edge_index2(0)),
				obj2.GetCollisionVertexVelocity(edge_index2(1))
			)
		);
	)

	// spdlog::info("inside step: {}, outside step: {}, final step: {}", max_step_inside, max_step_outside, max_step);

	if (max_step >= 1) {
		return max_step;
	} else {
		return max_step * 0.8;
	}
}