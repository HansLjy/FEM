//
// Created by hansljy on 12/1/22.
//

#ifndef FEM_COLLISIONUTILITY_H
#define FEM_COLLISIONUTILITY_H

#include "EigenAll.h"

int GetVFDistanceType(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3);
int GetEEDistanceType(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22);

double GetVLDistance(const Vector3d& vertex, const Vector3d& edge1, const Vector3d& edge2);
Vector9d GetVLDistanceGradient(const Vector3d& vertex, const Vector3d& edge1, const Vector3d& edge2);
Matrix9d GetVLDistanceHessian(const Vector3d& vertex, const Vector3d& edge1, const Vector3d& edge2);

double GetVVDistance(const Vector3d& v1, const Vector3d& v2);
Vector6d GetVVDistanceGradient(const Vector3d& v1, const Vector3d& v2);
Matrix6d GetVVDistanceHessian(const Vector3d &v1, const Vector3d &v2);

double GetVPDistance(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3);
Vector12d GetVPDistanceGradient(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3);
Matrix12d GetVPDistanceHessian(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3);

double GetLLDistance(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22);
Vector12d GetLLDistanceGradient(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22);
Matrix12d GetLLDistanceHessian(const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22);


double GetVFDistance(int distance_type, const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3);
double GetVFDistance(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3);
Vector12d GetVFDistanceGradient(int distance_type, const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3);
Vector12d GetVFDistanceGradient(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3);
Matrix12d GetVFDistanceHessian(int distance_type, const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3);
Matrix12d GetVFDistanceHessian(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3);

double GetEEDistance(int distance_type, const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22);
double GetEEDistance(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22);
Vector12d GetEEDistanceGradient(int distance_type, const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22);
Vector12d GetEEDistanceGradient(const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22);
Matrix12d GetEEDistanceHessian(int distance_type, const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22);
Matrix12d GetEEDistanceHessian(const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22);

#endif //FEM_COLLISIONUTILITY_H
