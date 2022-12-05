//
// Created by hansljy on 10/31/22.
//

#include "Newton.h"
#include "spdlog/spdlog.h"

void Newton::Optimize(const ValueFunc &f, const GradiantFunc &g, const HessianFunc &h, VectorXd &x) {
    VectorXd gradient;
    SparseMatrixXd hessian;
    g(x, gradient);
    h(x, hessian);
    Eigen::SimplicialLDLT<SparseMatrixXd> solver;

    int step = 0;
    while (step++ < _max_iter) {
        solver.compute(hessian);
        VectorXd p = solver.solve(-gradient);
        double alpha = 1;
        const double derivative = _c1 * gradient.dot(p);
        const double fk = f(x);
        while (f(x + alpha * p) > fk + alpha * derivative) {
            alpha /= 2;
        }
        x = x + alpha * p;
        g(x, gradient);
        if (gradient.norm() / gradient.size() < _tolerance) {
            break;
        }
        hessian.setZero();
        h(x, hessian);
    }
    if (step > 10) {
        spdlog::warn("Newton optimization not converge! Residue: {}", gradient.norm() / gradient.size());
    }
}