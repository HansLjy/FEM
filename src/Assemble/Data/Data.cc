#include "Data.hpp"
#include "ExternalForce/ExternalForceContainer.hpp"

template<>
Factory<ExternalForce<FixedObjectData>>* Factory<ExternalForce<FixedObjectData>>::_the_factory = nullptr;
