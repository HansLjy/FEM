//
// Created by hansljy on 11/9/22.
//

#ifndef FEM_DOMAININTEGRATOR_H
#define FEM_DOMAININTEGRATOR_H

#include "Integrator.h"
#include "Domain.h"

class DomainIntegrator : public Integrator {
public:
    explicit DomainIntegrator(const json& config);

    void Step(Target &target, double h) const override;
    void StepNonRoot(Domain& domain, double h) const;


protected:
    Integrator* _integrator;
};

#endif //FEM_DOMAININTEGRATOR_H
