//
// Created by hansljy on 11/17/22.
//

#ifndef FEM_INCREMENTALPOTENTIAL_H
#define FEM_INCREMENTALPOTENTIAL_H

#include "Integrator.h"
#include "Optimizer/Optimizer.h"

class IPIntegrator : public Integrator {
public:
    explicit IPIntegrator(const json& config);
    void Step(Target &target, double h) const override;

    ~IPIntegrator() override;

private:
    Optimizer* _optimizer;
};

#endif //FEM_INCREMENTALPOTENTIAL_H
