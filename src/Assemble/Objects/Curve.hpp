#pragma once

#include "Object.hpp"
#include "Data/CurveData.hpp"
#include "Coordinate/Coordinate.hpp"
#include "MassModel/MassModel.hpp"
#include "EnergyModel/EnergyModel.hpp"
#include "EnergyModel/CurveEnergyModel.hpp"
#include "Render/CurveShape.hpp"
#include "Collision/CollisionShape/CollisionShape.hpp"

using Curve = ConcreteObject<CurveData, CurveData, BasicCoordinate, SampledObjectMassModel, CurveEnergyModel, CurveShape, SampledCollisionShape>;
using BezierCurve = ConcreteObject<BezierCurveData, ReducedObjectData<CurveData>, ReducedCoordinate<BasicCoordinate>, ReducedMassModel<SampledObjectMassModel>, ReducedEnergyModel<CurveEnergyModel>, ProxiedRenderShape<CurveShape>, NullCollisionShape>;
