#include "TriangleData.hpp"
#include "ExternalForce/ExternalForce.hpp"

template<>
Factory<ExternalForce<TriangleData>>* Factory<ExternalForce<TriangleData>>::_the_factory = nullptr;