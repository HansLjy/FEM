//
// Created by hansljy on 10/31/22.
//

#ifndef FEM_NEWTON_H
#define FEM_NEWTON_H

#include "Optimizer.h"

class Newton : public Optimizer {
public:
    Newton(const json& config) : Newton(config["tolerance"], config["max-iteration"], config["wolfe-condition"]) {}
    Newton(double tolerance, int max_iter, double c1) : Optimizer(tolerance, max_iter), _c1(c1) {}

    void Optimize(const ValueFunc &f, const GradiantFunc &g, const HessianFunc &h, VectorXd &x) override;

protected:
    const double _c1;   // line search coefficient
};

#endif //FEM_NEWTON_H
