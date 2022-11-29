//
// Created by hansljy on 11/28/22.
//

#ifndef FEM_DOMAINDFSSTEPPER_H
#define FEM_DOMAINDFSSTEPPER_H

#include "TimeStepper.h"
#include "Domain.h"
#include "Integrator/Integrator.h"

class DomainDFSStepper : public TimeStepper {
public:
    explicit DomainDFSStepper(const json& config);
    void Step(double h) const override;

    ~DomainDFSStepper();

protected:
    void StepNonRoot(Domain& domain, double h) const;

    Integrator* _integrator;
};

#endif //FEM_DOMAINDFSSTEPPER_H
