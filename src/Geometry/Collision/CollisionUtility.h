//
// Created by hansljy on 12/1/22.
//

#ifndef FEM_COLLISIONUTILITY_H
#define FEM_COLLISIONUTILITY_H

#include "EigenAll.h"

double VertexFaceDistance(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3);
double EdgeEdgeDistance(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22);

#endif //FEM_COLLISIONUTILITY_H
