#ifndef PP_DISTANCE_H
#define PP_DISTANCE_H

#include "EigenAll.h"

namespace IPC {
    double point_point_distance(const Vector3d& a, const Vector3d& b);

    void point_point_distance_gradient(const Vector3d& a, const Vector3d& b,
        Vector6d& grad);

    void point_point_distance_hessian(const Vector3d& a, const Vector3d& b,
        Matrix6d& hess);

    void point_point_distance_diff_gradient(const Vector3d& a, const Vector3d& b, const Vector3d& ap, const Vector3d& bp,
        Vector6d& grad);

} // namespace Bow::Geometry::IPC

#endif