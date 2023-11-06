#include "PointPointDistance.h"

namespace IPC {
    double point_point_distance(const Vector3d& a, const Vector3d& b)
    {
        return (a - b).squaredNorm();
    }

    void point_point_distance_gradient(const Vector3d& a, const Vector3d& b,
        Vector6d& grad)
    {
        grad.template segment<3>(0) = 2.0 * (a - b);
        grad.template segment<3>(3) = -grad.template segment<3>(0);
    }

    void point_point_distance_hessian(const Vector3d& a, const Vector3d& b,
        Matrix6d& hess)
    {
        hess.setZero();
        hess.diagonal().setConstant(2.0);
        if constexpr (3 == 2) {
            hess(0, 2) = hess(1, 3) = hess(2, 0) = hess(3, 1) = -2.0;
        }
        else {
            hess(0, 3) = hess(1, 4) = hess(2, 5) = hess(3, 0) = hess(4, 1) = hess(5, 2) = -2.0;
        }
    }

    void point_point_distance_diff_gradient(const Vector3d& a, const Vector3d& b, const Vector3d& ap, const Vector3d& bp,
        Vector6d& grad)
    {
        grad.template segment<3>(0) = 2.0 * (ap - bp);
        grad.template segment<3>(3) = -grad.template segment<3>(0);
    }
}
