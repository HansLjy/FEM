//
// Created by hansljy on 11/28/22.
//

#ifndef FEM_SYSTEMSTEPPER_H
#define FEM_SYSTEMSTEPPER_H

#include "TimeStepper.h"
#include "Integrator/Integrator.h"

class SystemStepper : public TimeStepper {
public:
    explicit SystemStepper(const json& config);
    void Step(double h) const override;

    ~SystemStepper() override;

protected:
    Integrator* _integrator;
};

#endif //FEM_SYSTEMSTEPPER_H
