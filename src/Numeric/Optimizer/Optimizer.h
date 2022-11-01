//
// Created by hansljy on 10/31/22.
//

#ifndef FEM_OPTIMIZER_H
#define FEM_OPTIMIZER_H

#include <functional>
#include "EigenAll.h"

typedef std::function<double(const VectorXd&)> ValueFunc;
typedef std::function<VectorXd(const VectorXd&)> GradiantFunc;
typedef std::function<void(const VectorXd&, SparseMatrixXd&)> HessianFunc;

class Optimizer {
public:
    Optimizer(double tolerance, int max_iter) : _tolerance(tolerance), _max_iter(max_iter) {}
    virtual void Optimize(const ValueFunc& f, const GradiantFunc& g, VectorXd &x0) {
        throw std::logic_error("Unimplemented method");
    }

    // What a coincidence, g for gradiant, h for hessian
    virtual void Optimize(const ValueFunc &f, const GradiantFunc &g, const HessianFunc &h, VectorXd &x) {
        throw std::logic_error("Unimplemented method");
    }

protected:
    const double _tolerance;
    const int _max_iter;
};

#endif //FEM_OPTIMIZER_H
