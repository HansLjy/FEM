#ifndef Edge_Edge_Distance_H
#define Edge_Edge_Distance_H

#include "EigenAll.h"

namespace IPC {
    double edge_edge_cross_norm2(const Vector3d& ea0, const Vector3d& ea1, const Vector3d& eb0, const Vector3d& eb1);
    void edge_edge_cross_norm2_gradient(const Vector3d& ea0, const Vector3d& ea1, const Vector3d& eb0, const Vector3d& eb1, Vector12d& grad);
    void edge_edge_cross_norm2_hessian(const Vector3d& ea0, const Vector3d& ea1, const Vector3d& eb0, const Vector3d& eb1, Matrix12d& hessian);
    double edge_edge_mollifier_threshold(const Vector3d& ea0_rest, const Vector3d& ea1_rest, const Vector3d& eb0_rest, const Vector3d& eb1_rest);
    double edge_edge_mollifier(const Vector3d& ea0, const Vector3d& ea1, const Vector3d& eb0, const Vector3d& eb1, const double eps_x);
    void edge_edge_mollifier_gradient(const Vector3d& ea0, const Vector3d& ea1, const Vector3d& eb0, const Vector3d& eb1, const double eps_x, Vector12d& grad);
    void edge_edge_mollifier_hessian(const Vector3d& ea0, const Vector3d& ea1, const Vector3d& eb0, const Vector3d& eb1, const double eps_x, Matrix12d& hessian);
    double edge_edge_distance(const Vector3d& ea0, const Vector3d& ea1, const Vector3d& eb0, const Vector3d& eb1);
    void edge_edge_distance_gradient(const Vector3d& ea0, const Vector3d& ea1, const Vector3d& eb0, const Vector3d& eb1, Vector12d& grad);
    void edge_edge_distance_diff_gradient(const Vector3d& ea0, const Vector3d& ea1, const Vector3d& eb0, const Vector3d& eb1, const Vector3d& eap0, const Vector3d& eap1, const Vector3d& ebp0, const Vector3d& ebp1, Vector12d& grad);
    void edge_edge_distance_hessian(const Vector3d& ea0, const Vector3d& ea1, const Vector3d& eb0, const Vector3d& eb1, Matrix12d& hessian);

}

#endif