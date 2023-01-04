//
// Created by hansljy on 12/1/22.
//

#ifndef FEM_COLLISIONUTILITY_H
#define FEM_COLLISIONUTILITY_H

#include "EigenAll.h"

struct DistanceType {
    double _coef1;
    double _coef2;
};

/**
 * @return DistanceType
 *         face1 + coef1 * (face2 - face1) + coef2 * (face3 - face1) and vertex
 *         minimize the distance between vertex and face(the whole plane)
 */
inline DistanceType GetVFDistanceType(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) {
    Vector3d e1 = face2 - face1, e2 = face3 - face1, e3 = vertex - face1;
    double A = e1.dot(e1);
    double B = e1.dot(e2);
    double C = e2.dot(e2);
    double D = e3.dot(e1);
    double E = e3.dot(e2);
    double delta = A * C - B * B;

    return {(C * D - B * E) / delta, (-B * D + A * E) / delta };
}

/**
 * @return DistanceType 
 *         edge11 + coef1 * (edge12 - edge11) and edge21 + coef2 * (edge22 - edge21)
 *         minimize the distance between the two lines(not just the line segment)
 */
inline DistanceType GetEEDistanceType(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) {
    Vector3d e1 = edge12 - edge11;
    Vector3d e2 = edge22 - edge21;
    Vector3d e3 = edge21 - edge11;
    double A = e1.dot(e1);
    double B = e2.dot(e1);
    double C = e2.dot(e2);
    double D = e3.dot(e1);
    double E = e3.dot(e2);
    double delta = A * C - B * B;
    return {(C * D - B * E) / delta, (-B * D + A * E) / delta};
}


inline double GetVLDistance(const Vector3d& vertex, const Vector3d& edge1, const Vector3d& edge2) {
    return (vertex - edge1).cross(edge2 - edge1).norm() / (edge2 - edge1).norm();
}
inline Vector9d GetVLDistanceGradient(const Vector3d& vertex, const Vector3d& edge1, const Vector3d& edge2) {
    // TODO:
}
inline Matrix9d GetVLDistanceHessian(const Vector3d& vertex, const Vector3d& edge1, const Vector3d& edge2) {
    // TODO:
}

inline double GetVVDistance(const Vector3d& v1, const Vector3d& v2) {
    return (v1 - v2).norm();
}
inline Vector6d GetVVDistanceGradient(const Vector3d& v1, const Vector3d& v2) {
    // TODO:
}
inline Matrix6d GetVVDistanceHessian(const Vector3d &v1, const Vector3d &v2) {
    // TODO:
}

inline double GetVPDistance(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3, const double& lambda3, const double& lambda2) {
    return (vertex - (face1 + lambda3 * (face2 - face1) + lambda2 * (face3 - face1))).norm();
}
inline Vector12d GetVPDistanceGradient(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3, const double &lambda3, const double &lambda2) {
    // TODO:
}
inline Matrix12d GetVPDistanceHessian(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3, const double &lambda3, const double &lambda2) {
    // TODO:
}

inline double GetLLDistance(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22, const double& lambda1, const double& lambda2) {
    return ((edge11 + lambda1 * (edge12 - edge11)) - (edge21 + lambda2 * (edge22 - edge21))).norm();
}

inline Vector12d GetLLDistanceGradient(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22, const double& lambda1, const double& lambda2) {
    // TODO:
}

inline Matrix12d GetLLDistanceHessian(const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22, const double &lambda1, const double &lambda2) {
    // TODO:
}


double GetVFDistance(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3);
Vector12d GetVFDistanceGradient(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3);
Matrix12d GetVFDistanceHessian(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3);

double GetEEDistance(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22);
Vector12d GetEEDistanceGradient(const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22);
Matrix12d GetEEDistanceHessian(const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22);

#endif //FEM_COLLISIONUTILITY_H
