#include "CCD.h"
#include "SimpleCCD.h"

BEGIN_DEFINE_XXX_FACTORY(CCD)
    ADD_PRODUCT("simple-ccd", SimpleCCD)
END_DEFINE_XXX_FACTORY