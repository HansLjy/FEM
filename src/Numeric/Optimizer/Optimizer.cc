//
// Created by hansljy on 11/17/22.
//

#include "Optimizer.h"
#include "Newton.h"


BEGIN_DEFINE_XXX_FACTORY(Optimizer)
    ADD_PRODUCT("newton", Newton)
END_DEFINE_XXX_FACTORY