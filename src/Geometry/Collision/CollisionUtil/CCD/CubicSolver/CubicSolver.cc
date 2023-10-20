#include "CubicSolver.h"

template<>
Factory<CubicSolver>* Factory<CubicSolver>::_the_factory = nullptr;

#include "BisectionCubicSolver.h"
#include "CemCubicSolver.hpp"

const bool bisection_solver_registered = FactoryRegistration::RegisterForFactory<CubicSolver, BisectionCubicSolver>("bisection-solver");
const bool cem_solver_registered = FactoryRegistration::RegisterForFactory<CubicSolver, CemCubicSolver>("cem-solver");