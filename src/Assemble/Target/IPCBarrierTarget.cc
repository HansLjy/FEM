//
// Created by hansljy on 12/2/22.
//

#include "IPCBarrierTarget.h"
#include "Collision/Culling/CollisionCulling.h"
#include "Collision/CollisionUtility.h"

void IPCBarrierTarget::ComputeConstraintSet(const Eigen::VectorXd &x, int time_stamp) {
    _culling->ComputeConstraintSet(x, _objs, time_stamp, _d_hat, _constraint_set);
}

#define PROCESS_CONSTRAINT_PAIR(Initialization, VF, EE) \
    for (const auto& constraint_pair : _constraint_set) {\
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

double IPCBarrierTarget::GetBarrierEnergy() const {
    double barrier_energy = 0;
    PROCESS_CONSTRAINT_PAIR(
        ,
        barrier_energy += GetVFBarrierEnergy(vertex, face1, face2, face3);,
        barrier_energy += GetEEBarrierEnergy(edge11, edge12, edge21, edge22);
    )
    return barrier_energy;
}

VectorXd IPCBarrierTarget::GetBarrierEnergyGradient() const {
    VectorXd gradient(GetDOF());
    gradient.setZero();

    PROCESS_CONSTRAINT_PAIR (
        const auto& projection1 = obj1->GetVertexProjectionMatrix();
        const auto& projection2 = obj2->GetVertexProjectionMatrix();
        const int offset1 = _offsets[constraint_pair._obj_id1];
        const int offset2 = _offsets[constraint_pair._obj_id2];,

        const auto single_gradient = GetVFBarrierEnergyGradient(vertex, face1, face2, face3);
        gradient.segment(offset1, obj1->GetDOF())
            += projection1.middleRows(vertex_index * 3, 3).transpose() * single_gradient.segment(0, 3);
        gradient.segment(offset2, obj2->GetDOF())
            += projection2.middleRows(face_index(0) * 3, 3).transpose() * single_gradient.segment(3, 3)
             + projection2.middleRows(face_index(1) * 3, 3).transpose() * single_gradient.segment(6, 3)
             + projection2.middleRows(face_index(2) * 3, 3).transpose() * single_gradient.segment(9, 3);
        ,

        const auto single_gradient = GetEEBarrierEnergyGradient(edge11, edge12, edge21, edge22);
        gradient.segment(offset1, obj1->GetDOF())
            += projection1.middleRows(edge_index1(0) * 3, 3).transpose() * single_gradient.segment(0, 3)
             + projection1.middleRows(edge_index1(1) * 3, 3).transpose() * single_gradient.segment(3, 3);
        gradient.segment(offset2, obj2->GetDOF())
            += projection2.middleRows(edge_index2(0) * 3, 3).transpose() * single_gradient.segment(6, 3)
             + projection2.middleRows(edge_index2(1) * 3, 3).transpose() * single_gradient.segment(9, 3);
    )

    return gradient;
}

#define DISPERSE_SINGLE_HESSIAN \
    for (int i = 0, ii = 0; i < 4; i++, ii += 3) { \
        for (int j = 0, jj = 0; j < 4; j++, jj += 3) { \
            SparseToCOO(\
                projection[i] * single_hessian.block(ii, jj, 3, 3) * projection[j].transpose(),\
                coo, offset_x + offset[i], offset_y + offset[j]\
            );\
        }\
    }

void IPCBarrierTarget::GetBarrierEnergyHessian(COO &coo, int offset_x, int offset_y) const {
    for (const auto& constraint_pair : _constraint_set) {
        const auto obj1 = _objs[constraint_pair._obj_id1];
        const auto obj2 = _objs[constraint_pair._obj_id2];
        
        const auto& vertices1 = obj1->GetCollisionVertices();
        const auto& vertices2 = obj2->GetCollisionVertices();
        
        const auto& projection1 = obj1->GetVertexProjectionMatrix();
        const auto& projection2 = obj2->GetVertexProjectionMatrix();
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
                
                SparseMatrixXd single_hessian = GetVFBarrierEnergyHessian(vertex, face1, face2, face3).sparseView();
                
                const Ref<const SparseMatrixXd> projection[4] = {
                    projection1.middleRows(vertex_index * 3, 3).transpose(),
                    projection2.middleRows(face_index(0) * 3, 3).transpose(),
                    projection2.middleRows(face_index(1) * 3, 3).transpose(),
                    projection2.middleRows(face_index(2) * 3, 3).transpose()
                };
                const int offset[4] = {offset1, offset2, offset2, offset2};

                DISPERSE_SINGLE_HESSIAN

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
                
                SparseMatrixXd single_hessian = GetEEBarrierEnergyHessian(edge11, edge12, edge21, edge22).sparseView();

                const Ref<const SparseMatrixXd> projection[4] = {
                    projection1.middleRows(edge_index1(0) * 3, 3).transpose(),
                    projection1.middleRows(edge_index1(1) * 3, 3).transpose(),
                    projection2.middleRows(edge_index2(0) * 3, 3).transpose(),
                    projection2.middleRows(edge_index2(1) * 3, 3).transpose()
                };
                const int offset[4] = {offset1, offset1, offset2, offset2};

                DISPERSE_SINGLE_HESSIAN

                break;
            }
        }
    }
}

double IPCBarrierTarget::GetMaxStep(const Eigen::VectorXd &p) {
    return 1;
    // TODO
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
    return p2bpd2 * pdpx * pdpx.transpose() + pbpd * p2dpx2;
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
    return p2bpd2 * pdpx * pdpx.transpose() + pbpd * p2dpx2;
}
