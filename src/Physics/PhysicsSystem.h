//
// Created by hansljy on 11/10/22.
//

#ifndef FEM_PHYSICSSYSTEM_H
#define FEM_PHYSICSSYSTEM_H

#include "Target.h"
#include "ObjectCollection.h"

class PhysicsSystem : public Target, public ObjectCollection {};

DECLARE_XXX_FACTORY(PhysicsSystem)

#endif //FEM_PHYSICSSYSTEM_H
