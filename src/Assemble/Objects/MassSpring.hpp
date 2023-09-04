#pragma once

#include "Object.hpp"
#include "Data/MassSpringData.hpp"
#include "Coordinate/Coordinate.hpp"
#include "MassModel/MassModel.hpp"
#include "EnergyModel/MassSpringEnergyModel.hpp"
#include "Render/RenderShape.hpp"
#include "Collision/CollisionShape/CollisionShape.h"
#include "ExternalForce/SampledObjectFixtureForce.hpp"
#include "ExternalForce/SampledObjectGravity.hpp"

using MassSpring = ConcreteObject<MassSpringData, MassSpringData, BasicCoordinate, SampledObjectMassModel, MassSpringEnergyModel, SampledRenderShape, NullCollisionShape>;
