#ifndef PE_DISTANCE_H
#define PE_DISTANCE_H

#include "EigenAll.h"

namespace IPC {
    double point_edge_distance(const Vector3d& p, const Vector3d& e0, const Vector3d& e1);

    void point_edge_distance_gradient(
        const Vector3d& p,
        const Vector3d& e0,
        const Vector3d& e1,
        Vector9d& grad
    );

    void point_edge_distance_hessian(
        const Vector3d& p,
        const Vector3d& e0,
        const Vector3d& e1,
        Matrix9d& hess
    );

    void point_edge_distance_diff_gradient(
        const Vector3d& p,
        const Vector3d& e0,
        const Vector3d& e1,
        const Vector3d& pp,
        const Vector3d& ep0,
        const Vector3d& ep1,
        Vector9d& grad
    );

}

#endif