#pragma once

#include "Object.hpp"
#include "Data/ClothData.hpp"
#include "Coordinate/Coordinate.hpp"
#include "MassModel/MassModel.hpp"
#include "EnergyModel/EnergyModel.hpp"
#include "EnergyModel/ClothEnergyModel.hpp"
#include "Render/RenderShape.hpp"
#include "Collision/CollisionShape/CollisionShape.h"

using Cloth = ConcreteObject<ClothData, ClothData, BasicCoordinate, SampledObjectMassModel, ClothEnergyModel, SampledRenderShape, SampledCollisionShape>;
using BezierCloth = ConcreteObject<BezierClothData, ReducedObjectData<ClothData>, ReducedCoordinate<BasicCoordinate>, ReducedMassModel<SampledObjectMassModel>, ReducedEnergyModel<ClothEnergyModel>, ProxiedRenderShape<SampledRenderShape>, NullCollisionShape>;
