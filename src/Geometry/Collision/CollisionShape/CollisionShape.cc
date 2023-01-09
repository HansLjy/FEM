#include "CollisionShape.h"
#include "FixedCollisionShape/RectangleCollisionShape.h"

BEGIN_DEFINE_XXX_FACTORY(FixedCollisionShape)
    ADD_PRODUCT("rectangle", RectangleCollisionShape)
END_DEFINE_XXX_FACTORY