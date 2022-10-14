//
// Created by hansljy on 10/14/22.
//

#include "ExternalForce.h"
#include "Curve/CurveGravity.h"

BEGIN_DEFINE_XXX_FACTORY(ExternalForce)
    ADD_PRODUCT("curve-gravity", CurveGravity)
END_DEFINE_XXX_FACTORY