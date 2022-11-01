//
// Created by hansljy on 10/31/22.
//

#include "Newton.h"

void Newton::Optimize(const ValueFunc &f, const GradiantFunc &g, const HessianFunc &h, VectorXd &x) {
    int step = 0;
    VectorXd gradient = g(x);
    SparseMatrixXd hessian;
    h(x, hessian);
    Eigen::SimplicialLDLT<SparseMatrixXd> LDLT(hessian);
    while (step < _max_iter && gradient.norm() > _tolerance) {
        VectorXd p = LDLT.solve(-gradient);
        double alpha = 1;
        const double derivative = _c1 * gradient.dot(p);
        const double fk = f(x);
        while (f(x + alpha * p) > fk + alpha * derivative) {
            alpha /= 2;
        }
        x = x + alpha * p;
    }
}