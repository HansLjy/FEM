#include "CubicSolver.h"
#include "BisectionCubicSolver.h"

BEGIN_DEFINE_XXX_FACTORY(CubicSolver)
    ADD_PRODUCT("bisection-solver", BisectionCubicSolver)
END_DEFINE_XXX_FACTORY