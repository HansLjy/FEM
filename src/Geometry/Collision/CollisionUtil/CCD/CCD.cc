#include "CCD.h"

template<>
Factory<CCD>* Factory<CCD>::_the_factory = nullptr;

#include "SimpleCCD.h"
const bool simple_ccd_registered = FactoryRegistration::RegisterForFactory<CCD, SimpleCCD>("simple-ccd");
