//
// Created by hansljy on 11/28/22.
//

#include "TimeStepper.h"
#include "TimeStepper/SystemStepper.h"
#include "TimeStepper/BFSStepper.h"

void TimeStepper::Bind(System &system) {
    _system = &system;
}

BEGIN_DEFINE_XXX_FACTORY(TimeStepper)
    ADD_PRODUCT("system-stepper", SystemStepper)
    ADD_PRODUCT("bfs-stepper", BFSStepper)
    ADD_PRODUCT("parallel-bfs-stepper", ParallelBFSStepper)
END_DEFINE_XXX_FACTORY