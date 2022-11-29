//
// Created by hansljy on 11/29/22.
//

#ifndef FEM_DOMAINBFSSTEPPER_H
#define FEM_DOMAINBFSSTEPPER_H

#include "TimeStepper.h"
#include "Integrator/Integrator.h"

class Domain;

class DomainBFSStepper : public TimeStepper {
public:
    explicit DomainBFSStepper(const json& config);
    void Bind(System &system) override;
    void Step(double h) const override;

    ~DomainBFSStepper() noexcept override;

protected:
    Integrator* _integrator;
    std::vector<Domain*> _domains;
    std::vector<int> _level_bar;
};

#endif //FEM_DOMAINBFSSTEPPER_H
