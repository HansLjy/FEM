#ifndef PT_DISTANCE_H
#define PT_DISTANCE_H

#include "EigenAll.h"

namespace IPC {

    double point_triangle_distance(
        const Vector3d& p,
        const Vector3d& t0, const Vector3d& t1, const Vector3d& t2
    );

    void point_triangle_distance_gradient(
        const Vector3d& p,
        const Vector3d& t0, const Vector3d& t1, const Vector3d& t2,
        Vector12d& grad);

    void point_triangle_distance_diff_gradient(
        const Vector3d& p, const Vector3d& t0, const Vector3d& t1, const Vector3d& t2,
        const Vector3d& pp, const Vector3d& tp0, const Vector3d& tp1, const Vector3d& tp2,
        Vector12d& grad
    );

    void point_triangle_distance_hessian(
        const Vector3d& p,
        const Vector3d& t0, const Vector3d& t1, const Vector3d& t2,
        Matrix12d& hess
    );

} // namespace Bow::Geometry::IPC

#endif