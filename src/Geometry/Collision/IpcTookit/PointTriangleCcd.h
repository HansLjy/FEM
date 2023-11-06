#ifndef POINT_TRIANGLE_CCD_H
#define POINT_TRIANGLE_CCD_H

#include "EigenAll.h"

namespace IPC {
    bool point_triangle_cd_broadphase(
        const Vector3d& p,
        const Vector3d& t0,
        const Vector3d& t1,
        const Vector3d& t2,
        double dist
    );

    bool point_triangle_ccd_broadphase(
        const Vector3d& p,
        const Vector3d& t0,
        const Vector3d& t1,
        const Vector3d& t2,
        const Vector3d& dp,
        const Vector3d& dt0,
        const Vector3d& dt1,
        const Vector3d& dt2,
        double dist
    );

    double point_triangle_ccd(
        const Vector3d& p,
        const Vector3d& t0,
        const Vector3d& t1,
        const Vector3d& t2,
        const Vector3d& dp,
        const Vector3d& dt0,
        const Vector3d& dt1,
        const Vector3d& dt2,
        double eta, double thickness
    );

    double point_triangle_ccd2(
        int tid,
        const Vector3d& p,
        const Vector3d& t0,
        const Vector3d& t1,
        const Vector3d& t2,
        const Vector3d& dp,
        const Vector3d& dt0,
        const Vector3d& dt1,
        const Vector3d& dt2,
        double eta, double thickness
    );
} // namespace Bow::Geometry::IPC

#endif