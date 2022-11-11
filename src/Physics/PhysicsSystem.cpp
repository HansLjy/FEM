//
// Created by hansljy on 11/10/22.
//

#include "PhysicsSystem.h"
#include "InertialSystem.h"
#include "Domain.h"
#include "Tree/TreeDomain.h"

BEGIN_DEFINE_XXX_FACTORY(PhysicsSystem)
    ADD_PRODUCT("inertial-system", InertialSystem)
    ADD_PRODUCT("tree-domain", TreeDomain)
END_DEFINE_XXX_FACTORY