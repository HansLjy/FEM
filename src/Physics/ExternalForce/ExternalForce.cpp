//
// Created by hansljy on 10/14/22.
//

#include "ExternalForce.h"
#include "SampledObject/SampledObjectGravity.h"

BEGIN_DEFINE_XXX_FACTORY(ExternalForce)
    ADD_PRODUCT("sampled-object-gravity", SampledObjectGravity)
END_DEFINE_XXX_FACTORY