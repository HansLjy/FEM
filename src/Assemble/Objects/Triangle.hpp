#pragma once

#include "Object.hpp"
#include "Data/TriangleData.hpp"
#include "Coordinate/Coordinate.hpp"
#include "EnergyModel/EnergyModel.hpp"
#include "EnergyModel/FixtureEnergyModel.hpp"
#include "EnergyModel/MassSpringEnergyModel.hpp"
#include "MassModel/MassModel.hpp"
#include "Render/RenderShape.hpp"
#include "Collision/CollisionShape/CollisionShape.h"

using Triangle = ConcreteObject<TriangleData, TriangleData, BasicCoordinate, SampledObjectMassModel, MassSpringEnergyModel, SampledRenderShape, SampledCollisionShape>;