//
// Created by hansljy on 11/29/22.
//

#ifndef FEM_DOMAINBFSSTEPPER_H
#define FEM_DOMAINBFSSTEPPER_H

#include "TimeStepper.h"
#include "Integrator/Integrator.h"

class BFSStepper : public TimeStepper {
public:
    explicit BFSStepper(const json& config);
    void Bind(System &system) override;
    void Step(double h) const override;

    ~BFSStepper() noexcept override;

protected:
    Integrator* _integrator;
	int _levels;
    std::vector<Target*> _level_targets;
};

#endif //FEM_DOMAINBFSSTEPPER_H
