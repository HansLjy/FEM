#include "TimeStepper.hpp"
#include "Pattern.h"

template<>
Factory<TimeStepper>* Factory<TimeStepper>::_the_factory = nullptr;

#include "IncrementalPotential.hpp"
#include "ProjectiveDynamics.hpp"
#include "PDIPC.hpp"

const bool ip_registered = FactoryRegistration::RegisterForFactory<TimeStepper, IncrementalPotentialTimeStepper>("incremental-potential");
const bool pd_registerd = FactoryRegistration::RegisterForFactory<TimeStepper, ProjectiveDynamics>("projective-dynamics");
const bool pdipc_registered = FactoryRegistration::RegisterForFactory<TimeStepper, PDIPC>("PDIPC");
