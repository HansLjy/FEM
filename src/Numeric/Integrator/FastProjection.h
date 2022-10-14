//
// Created by hansljy on 10/8/22.
//

#ifndef FEM_FASTPROJECTION_H
#define FEM_FASTPROJECTION_H

#include "Integrator.h"

class FastProjectionIntegrator final : public Integrator {
public:
    explicit FastProjectionIntegrator(const json& config);
    FastProjectionIntegrator(double tolerance, int max_step);
    void Step(Target &target, double h, VectorXd &x_next, VectorXd &v_next) const override;

private:
    double _tolerance;
    int _max_step;
};

#endif //FEM_FASTPROJECTION_H
