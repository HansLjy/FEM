#ifndef EDGE_EDGE_CCD_H
#define EDGE_EDGE_CCD_H

#include "EigenAll.h"

namespace IPC {

    bool edge_edge_cd_broadphase(
        const Vector3d& ea0,
        const Vector3d& ea1,
        const Vector3d& eb0,
        const Vector3d& eb1,
        double dist);

    bool edge_edge_ccd_broadphase(
        const Vector3d& ea0,
        const Vector3d& ea1,
        const Vector3d& eb0,
        const Vector3d& eb1,
        const Vector3d& dea0,
        const Vector3d& dea1,
        const Vector3d& deb0,
        const Vector3d& deb1,
        double dist);

    double edge_edge_ccd(
        const Vector3d& ea0,
        const Vector3d& ea1,
        const Vector3d& eb0,
        const Vector3d& eb1,
        const Vector3d& dea0,
        const Vector3d& dea1,
        const Vector3d& deb0,
        const Vector3d& deb1,
        double eta, double thickness
    );
}

#endif