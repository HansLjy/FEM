//
// Created by hansljy on 10/31/22.
//

#include "Newton.h"
#include "spdlog/spdlog.h"

void Newton::Optimize(const ValueFunc &f, const GradiantFunc &g, const HessianFunc &h, VectorXd &x) {
    VectorXd gradient = g(x);
    SparseMatrixXd hessian;
    h(x, hessian);
    Eigen::SimplicialLDLT<SparseMatrixXd> solver;

    int step = 0;
    while (step++ < _max_iter && gradient.norm() > _tolerance) {
        solver.compute(hessian);
        VectorXd p = solver.solve(-gradient);
        double alpha = 1;
        const double derivative = _c1 * gradient.dot(p);
        const double fk = f(x);
        while (f(x + alpha * p) > fk + alpha * derivative) {
            alpha /= 2;
        }
        x = x + alpha * p;
        gradient = g(x);
        hessian.resize(0, 0);   // clear hessian
        h(x, hessian);
    }
//    spdlog::info("Finished optimization in {} steps", step);
}