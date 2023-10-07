//
// Created by hansljy on 12/2/22.
//

#include "IPCHelper.hpp"
#include "Collision/CollisionUtility.h"
#include "spdlog/spdlog.h"
#include "Collision/CollisionShape/CollisionShape.hpp"

template<>
Factory<IPCHelper>* Factory<IPCHelper>::_the_factory = nullptr;

IPCHelper::IPCHelper(const json& config)
    : _d_hat(config["d-hat"]),
	  _kappa(config["kappa"]) {}

#define PROCESS_CONSTRAINT_PAIR(Initialization, VF, EE) \
    for (const auto& constraint_pair : _constraint_set) {\
		const int id1 = constraint_pair._obj_id1;\
		const int id2 = constraint_pair._obj_id2;\
        const auto obj1 = _objs[constraint_pair._obj_id1];\
        const auto obj2 = _objs[constraint_pair._obj_id2];\
		\
        const auto& vertices1 = obj1->GetCollisionVertices();\
        const auto& vertices2 = obj2->GetCollisionVertices();\
        \
        Initialization \
        switch (constraint_pair._type) {\
            case CollisionType::kVertexFace: {\
                const int vertex_index = constraint_pair._primitive_id1;\
                const RowVector3i face_index = obj2->GetCollisionFaceTopo().row(constraint_pair._primitive_id2);\
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
                    edge_index1 = obj1->GetCollisionEdgeTopo().row(constraint_pair._primitive_id1),\
                    edge_index2 = obj2->GetCollisionEdgeTopo().row(constraint_pair._primitive_id2);\
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

double IPCHelper::GetBarrierEnergy() const {
    double barrier_energy = 0;
    PROCESS_CONSTRAINT_PAIR(
        ,
        barrier_energy += GetVFBarrierEnergy(vertex, face1, face2, face3);,
        barrier_energy += GetEEBarrierEnergy(edge11, edge12, edge21, edge22);
    )
    return barrier_energy * _kappa;
}

VectorXd IPCHelper::GetBarrierEnergyGradient() const {
    VectorXd gradient(_dof);
    gradient.setZero();

    PROCESS_CONSTRAINT_PAIR (
        const int dof1 = _dofs[id1];
        const int dof2 = _dofs[id2];
        const int offset1 = _offsets[constraint_pair._obj_id1];
        const int offset2 = _offsets[constraint_pair._obj_id2];,

        const Vector12d single_gradient = GetVFBarrierEnergyGradient(vertex, face1, face2, face3);
        obj1->GetCollisionVertexDerivative(vertex_index).RightProduct(single_gradient.segment<3>(0), gradient.segment(offset1, dof1));
        obj2->GetCollisionVertexDerivative(face_index(0)).RightProduct(single_gradient.segment<3>(3), gradient.segment(offset2, dof2));
        obj2->GetCollisionVertexDerivative(face_index(1)).RightProduct(single_gradient.segment<3>(6), gradient.segment(offset2, dof2));
        obj2->GetCollisionVertexDerivative(face_index(2)).RightProduct(single_gradient.segment<3>(9), gradient.segment(offset2, dof2));
        ,

        const Vector12d single_gradient = GetEEBarrierEnergyGradient(edge11, edge12, edge21, edge22);
        obj1->GetCollisionVertexDerivative(edge_index1(0)).RightProduct(single_gradient.segment<3>(0), gradient.segment(offset1, dof1));
        obj1->GetCollisionVertexDerivative(edge_index1(1)).RightProduct(single_gradient.segment<3>(3), gradient.segment(offset1, dof1));
        obj2->GetCollisionVertexDerivative(edge_index2(0)).RightProduct(single_gradient.segment<3>(6), gradient.segment(offset2, dof2));
        obj2->GetCollisionVertexDerivative(edge_index2(1)).RightProduct(single_gradient.segment<3>(9), gradient.segment(offset2, dof2));
    )

    return gradient * _kappa;
}

void IPCHelper::GetBarrierEnergyHessian(COO &coo, int offset_x, int offset_y) const {
    for (const auto& constraint_pair : _constraint_set) {
        const auto obj1 = _objs[constraint_pair._obj_id1];
        const auto obj2 = _objs[constraint_pair._obj_id2];

        const auto& vertices1 = obj1->GetCollisionVertices();
        const auto& vertices2 = obj2->GetCollisionVertices();
        
        const int offset1 = _offsets[constraint_pair._obj_id1];
        const int offset2 = _offsets[constraint_pair._obj_id2];

        switch (constraint_pair._type) {
            case CollisionType::kVertexFace: {
                const int vertex_index = constraint_pair._primitive_id1;
                const RowVector3i face_index = obj2->GetCollisionFaceTopo().row(constraint_pair._primitive_id2);
                const Vector3d
                    vertex = vertices1.row(vertex_index),
                    face1 = vertices2.row(face_index(0)),
                    face2 = vertices2.row(face_index(1)),
                    face3 = vertices2.row(face_index(2));
                
                Matrix12d single_hessian = _kappa * GetVFBarrierEnergyHessian(vertex, face1, face2, face3);
                
                const int offset[4] = {offset1, offset2, offset2, offset2};
				const int index[4] = {vertex_index, face_index(0), face_index(1), face_index(2)};
                const CollisionShapeInterface* shape[4] = {obj1, obj2, obj2, obj2};

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
                    edge_index1 = obj1->GetCollisionEdgeTopo().row(constraint_pair._primitive_id1),
                    edge_index2 = obj2->GetCollisionEdgeTopo().row(constraint_pair._primitive_id2);
                const Vector3d
                    edge11 = vertices1.row(edge_index1(0)),
                    edge12 = vertices1.row(edge_index1(1)),
                    edge21 = vertices2.row(edge_index2(0)),
                    edge22 = vertices2.row(edge_index2(1));
                
                Matrix12d single_hessian = _kappa * GetEEBarrierEnergyHessian(edge11, edge12, edge21, edge22);

                const int offset[4] = {offset1, offset1, offset2, offset2};
				const int index[4] = {edge_index1(0), edge_index1(1), edge_index2(0), edge_index2(1)};
                const CollisionShapeInterface* shape[4] = {obj1, obj1, obj2, obj2};

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

double IPCHelper::GetVFBarrierEnergy(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const {
    double d = GetVFDistance(vertex, face1, face2, face3);
    return -(d - _d_hat) * (d - _d_hat) * log(d / _d_hat);
}

Vector12d IPCHelper::GetVFBarrierEnergyGradient(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const {
    double d = GetVFDistance(vertex, face1, face2, face3);
    Vector12d pdpx = GetVFDistanceGradient(vertex, face1, face2, face3);

    double pbpd = -2 * (d - _d_hat) * log(d / _d_hat) - (d - _d_hat) * (d - _d_hat) / d;
    return  pbpd * pdpx;
}

Matrix12d IPCHelper::GetVFBarrierEnergyHessian(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const {
    double d = GetVFDistance(vertex, face1, face2, face3);
    Vector12d pdpx = GetVFDistanceGradient(vertex, face1, face2, face3);
    Matrix12d p2dpx2 = GetVFDistanceHessian(vertex, face1, face2, face3);
    
    const double pbpd = -2 * (d - _d_hat) * log(d / _d_hat) - (d - _d_hat) * (d - _d_hat) / d;
    const double p2bpd2 = -2 * log(d / _d_hat) - 4 * (d - _d_hat) / d + (d - _d_hat) * (d - _d_hat) / (d * d);
    return PositiveProject<12>(p2bpd2 * pdpx * pdpx.transpose() + pbpd * p2dpx2);
}

double IPCHelper::GetEEBarrierEnergy(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const {
    double d = GetEEDistance(edge11, edge12, edge21, edge22);
    return -(d - _d_hat) * (d - _d_hat) * log(d / _d_hat);
}

Vector12d IPCHelper::GetEEBarrierEnergyGradient(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const {
    double d = GetEEDistance(edge11, edge12, edge21, edge22);
    Vector12d pdpx = GetEEDistanceGradient(edge11, edge12, edge21, edge22);

    double pbpd = -2 * (d - _d_hat) * log(d / _d_hat) - (d - _d_hat) * (d - _d_hat) / d;
    return  pbpd * pdpx;    
}

Matrix12d IPCHelper::GetEEBarrierEnergyHessian(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const {
    double d = GetEEDistance(edge11, edge12, edge21, edge22);
    Vector12d pdpx = GetEEDistanceGradient(edge11, edge12, edge21, edge22);
    Matrix12d p2dpx2 = GetEEDistanceHessian(edge11, edge12, edge21, edge22);
    
    const double pbpd = -2 * (d - _d_hat) * log(d / _d_hat) - (d - _d_hat) * (d - _d_hat) / d;
    const double p2bpd2 = -2 * log(d / _d_hat) - 4 * (d - _d_hat) / d + (d - _d_hat) * (d - _d_hat) / (d * d);
    return PositiveProject<12>(p2bpd2 * pdpx * pdpx.transpose() + pbpd * p2dpx2);
}