#pragma once

#include "Object.hpp"
#include "Data/ReconstructedMeshData.hpp"
#include "Coordinate/Coordinate.hpp"
#include "MassModel/MassModel.hpp"
#include "EnergyModel/EnergyModel.hpp"
#include "EnergyModel/ClothEnergyModel.hpp"
#include "Render/RenderShape.hpp"
#include "Collision/CollisionShape/CollisionShape.hpp"

using ReconstructedMesh = ConcreteObject<ReconstructedMeshData, ReconstructedMeshData, BasicCoordinate, SampledObjectMassModel, ClothEnergyModel, SampledRenderShape, NullCollisionShape>;