//
// Created by hansljy on 11/28/22.
//

#include "TimeStepper.h"
#include "TimeStepper/SystemStepper.h"
#include "TimeStepper/DomainDFSStepper.h"
#include "TimeStepper/DomainBFSStepper.h"

void TimeStepper::Bind(System &system) {
    _system = &system;
}

BEGIN_DEFINE_XXX_FACTORY(TimeStepper)
    ADD_PRODUCT("system-stepper", SystemStepper)
    ADD_PRODUCT("domain-dfs-stepper", DomainDFSStepper)
    ADD_PRODUCT("domain-bfs-stepper", DomainBFSStepper)
END_DEFINE_XXX_FACTORY