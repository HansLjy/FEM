//
// Created by hansljy on 10/13/22.
//

#include "Integrator.h"
#include "FastProjection.h"
#include "IncrementalPotential.h"

BEGIN_DEFINE_XXX_FACTORY(Integrator)
    ADD_PRODUCT("fast-projection", FastProjectionIntegrator)
    ADD_PRODUCT("incremental-potential", IPIntegrator)
END_DEFINE_XXX_FACTORY