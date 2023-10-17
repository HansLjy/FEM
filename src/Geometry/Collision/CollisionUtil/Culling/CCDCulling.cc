#include "CCDCulling.hpp"
#include "Pattern.h"

template<>
Factory<CCDCulling>* Factory<CCDCulling>::_the_factory = nullptr;

#include "SpatialHashingCulling.hpp"

const bool sphs_culling_registered = FactoryRegistration::RegisterForFactory<CCDCulling, SpatialHashingCulling>("spatial-hashing");