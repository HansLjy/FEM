#include "CollisionInterface.hpp"
#include "Object.hpp"

template<>
Caster<CollisionInterface>* Caster<CollisionInterface>::_the_factory = nullptr;