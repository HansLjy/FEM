//
// Created by hansljy on 10/8/22.
//

#ifndef FEM_INTEGRATOR_H
#define FEM_INTEGRATOR_H

#include "Constraint/Constraint.h"
#include "Pattern.h"
#include "Target.h"
#include "spdlog/spdlog.h"

class Integrator {
public:
    virtual void Step(Target &target, double h) const = 0;
    virtual ~Integrator() = default;
};

DECLARE_XXX_FACTORY(Integrator)

#endif //FEM_INTEGRATOR_H
