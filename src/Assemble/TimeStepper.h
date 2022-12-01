//
// Created by hansljy on 11/28/22.
//

#ifndef FEM_TIMESTEPPER_H
#define FEM_TIMESTEPPER_H

#include "System.h"

class TimeStepper {
public:
    virtual void Bind(System& system);
    virtual void Step(double h) const = 0;
    virtual ~TimeStepper() = default;

protected:
    System* _system = nullptr;
};

DECLARE_XXX_FACTORY(TimeStepper)

#endif //FEM_TIMESTEPPER_H
