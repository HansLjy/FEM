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
    ~DomainIntegrator() override;

protected:
    void StepNonRoot(DomainTarget &domain_target, double h) const;
    Integrator* _integrator;
};

#endif //FEM_DOMAININTEGRATOR_H
