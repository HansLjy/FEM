//
// Created by hansljy on 12/1/22.
//

#include "CollisionUtility.h"

double VertexFaceDistance(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) {
    Vector3d face_area = (face2 - face1).cross(face3 - face1);
    return std::abs(
        face_area.dot(vertex - face1)
    ) / face_area.norm();
}

double EdgeEdgeDistance(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) {
    Vector3d area = (edge12 - edge11).cross(edge22 - edge21);
    return std::abs(
        area.dot(edge21 - edge11)
    ) / area.norm();
}