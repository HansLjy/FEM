//
// Created by hansljy on 11/29/22.
//

#ifndef FEM_DOMAINBFSSTEPPER_H
#define FEM_DOMAINBFSSTEPPER_H

#include "TimeStepper.h"
#include "Integrator/Integrator.h"

class DomainBFSStepper : public TimeStepper {
public:
    explicit DomainBFSStepper(const json& config);
    void Step(double h) const override;

    ~DomainBFSStepper() noexcept override;

protected:
    Integrator* _integrator;
};

#endif //FEM_DOMAINBFSSTEPPER_H
