#include "CubicSolver.h"

template<>
Factory<CubicSolver>* Factory<CubicSolver>::_the_factory = nullptr;

#include "BisectionCubicSolver.h"

const bool bisection_solver_registered = FactoryRegistration::RegisterForFactory<CubicSolver, BisectionCubicSolver>("bisection-solver");