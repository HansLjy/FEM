#ifndef POINT_EDGE_CCD_H
#define POINT_EDGE_CCD_H

#include "EigenAll.h"

namespace IPC {

    bool point_edge_cd_broadphase(
        const Vector3d& x0,
        const Vector3d& x1,
        const Vector3d& x2,
    double dist);

    bool point_edge_ccd_broadphase(
        const Vector3d& p,
        const Vector3d& e0,
        const Vector3d& e1,
        const Vector3d& dp,
        const Vector3d& de0,
        const Vector3d& de1,
        const double dist
    );

    // double point_edge_ccd(
    //     const Vector2d& x0,
    //     const Vector2d& x1,
    //     const Vector2d& x2,
    //     const Vector2d& d0,
    //     const Vector2d& d1,
    //     const Vector2d& d2,
    //     double eta
    // );
} // namespace Bow::Geometry::IPC


#endif