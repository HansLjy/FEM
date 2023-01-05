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
    const Vector3d e1 = vertex - edge1;
    const Vector3d e2 = edge2 - edge1;
    const double d = e1.cross(e2).norm() / e2.norm();

    const double A = e1.dot(e1);
    const double B = e1.dot(e2);
    const double C = e2.dot(e2);

    const double pspA = 1;
    const double pspB = - 2 * B / C;
    const double pspC = B * B / (C * C);
    
    return 1.0 / (2 * d) * (Vector9d() <<
        pspA * 2 * e1 + pspB * e2,
        - pspA * 2 * e1 + pspB * (- e1 - e2) - pspC * 2 * e2,
        pspB * e1 + pspC * 2 * e2
    ).finished();
}

inline Matrix9d GetVLDistanceHessian(const Vector3d& vertex, const Vector3d& edge1, const Vector3d& edge2) {
    const Vector3d e1 = vertex - edge1;
    const Vector3d e2 = edge2 - edge1;
    const double d = e1.cross(e2).norm() / e2.norm();

    const double A = e1.dot(e1);
    const double B = e1.dot(e2);
    const double C = e2.dot(e2);

    const double pspA = 1;
    const double pspB = - 2 * B / C;
    const double pspC = B * B / (C * C);

    Vector9d pspx = (Vector9d() <<
        pspA * 2 * e1 + pspB * e2,
        - pspA * 2 * e1 + pspB * (- e1 - e2) - pspC * 2 * e2,
        pspB * e1 + pspC * 2 * e2
    ).finished();
    
    Matrix2d p2spy2 = (Matrix2d() << 
        - 2 / C, 2 * B / (C * C),
        2 * B / (C * C), -2 * B * B / (C * C * C)
    ).finished();

    Matrix<double, 9, 2> pypx;
    pypx.block<3, 1>(0, 0) = e2;
    pypx.block<3, 1>(3, 0) = vertex - edge2;
    pypx.block<3, 1>(6, 0) = e1;
    pypx.block<3, 1>(0, 1) = Vector3d::Zero();
    pypx.block<3, 1>(3, 1) = -2 * e2;
    pypx.block<3, 1>(6, 1) = 2 * e2;

    Matrix9d p2spx2 = pypx * p2spy2 * pypx.transpose();
    p2spx2.block<3, 3>(0, 0) += (2 * pspA) * Matrix3d::Identity();
    p2spx2.block<3, 3>(0, 3) += (-2 * pspA - pspB) * Matrix3d::Identity();
    p2spx2.block<3, 3>(3, 0) += (-2 * pspA - pspB) * Matrix3d::Identity();
    p2spx2.block<3, 3>(0, 6) += pspB * Matrix3d::Identity();
    p2spx2.block<3, 3>(6, 0) += pspB * Matrix3d::Identity();
    p2spx2.block<3, 3>(3, 3) += 2 * (pspA + pspB + pspC) * Matrix3d::Identity();
    p2spx2.block<3, 3>(3, 6) += (-pspB - 2 * pspC) * Matrix3d::Identity();
    p2spx2.block<3, 3>(6, 3) += (-pspB - 2 * pspC) * Matrix3d::Identity();
    p2spx2.block<3, 3>(6, 6) += 2 * pspC * Matrix3d::Identity();

    return 1.0 / (2 * d) * p2spx2 - 1.0 / (4 * d * d * d) * pspx * pspx.transpose();
}

inline double GetVVDistance(const Vector3d& v1, const Vector3d& v2) {
    return (v1 - v2).norm();
}
inline Vector6d GetVVDistanceGradient(const Vector3d& v1, const Vector3d& v2) {
    double d = (v1 - v2).norm();
    return 1.0 / (2 * d) * (Vector6d() << 2 * v1 - 2 * v2, -2 * v1 + 2 * v2).finished();
}

inline Matrix6d GetVVDistanceHessian(const Vector3d &v1, const Vector3d &v2) {
    static const Matrix6d p2spx2 = (Matrix6d() <<
        2, 0, 0, -2, 0, 0,
        0, 2, 0, 0, -2, 0,
        0, 0, 2, 0, 0, -2,
        -2, 0, 0, 2, 0, 0,
        0, -2, 0, 0, 2, 0,
        0, 0, -2, 0, 0, 2
    ).finished();

    double d = (v1 - v2).norm();
    Vector6d pspx = (Vector6d() << 2 * v1 - 2 * v2, -2 * v1 + 2 * v2).finished();
    
    return 1.0 / (2 * d) * p2spx2 - 1 / (4 * d * d * d) * pspx * pspx.transpose();
}

inline double GetVPDistance(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3, const double& lambda3, const double& lambda2) {
    return (vertex - (face1 + lambda3 * (face2 - face1) + lambda2 * (face3 - face1))).norm();
}

Vector12d GetVPDistanceGradient(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3, const double &lambda3, const double &lambda2);
Matrix12d GetVPDistanceHessian(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3, const double &lambda3, const double &lambda2);

inline double GetLLDistance(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22, const double& lambda1, const double& lambda2) {
    return ((edge11 + lambda1 * (edge12 - edge11)) - (edge21 + lambda2 * (edge22 - edge21))).norm();
}

Vector12d GetLLDistanceGradient(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22, const double& lambda1, const double& lambda2);
Matrix12d GetLLDistanceHessian(const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22, const double &lambda1, const double &lambda2);


double GetVFDistance(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3);
Vector12d GetVFDistanceGradient(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3);
Matrix12d GetVFDistanceHessian(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3);

double GetEEDistance(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22);
Vector12d GetEEDistanceGradient(const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22);
Matrix12d GetEEDistanceHessian(const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22);

#endif //FEM_COLLISIONUTILITY_H
