#include "Object.hpp"
#include "Data/CoarseGridBasedData.hpp"
#include "Model/MassSpringModel.hpp"
#include "Model/GridBasedPhysics.hpp"
#include "Render/GridBasedShape.hpp"
#include "Collision/CollisionShape/CollisionShape.h"
#include "ExternalForce/SampledObjectGravity.hpp"
#include "ExternalForce/SampledObjectFixtureForce.hpp"

using CoarseGrid = ConcreteObject<CoarseGridBasedData, CoarseGridBasedData, GridBasedPhysics, GridBasedShape, NullCollisionShape>;

template<>
Factory<ExternalForce<CoarseGridBasedData>>* Factory<ExternalForce<CoarseGridBasedData>>::_the_factory = nullptr;

namespace {
	const bool coarse_grid_registered = Factory<Object>::GetInstance()->Register("coarse-grid", [](const json& config) {
		return new CoarseGrid(config);
	});
	const bool gravity_registered = Factory<ExternalForce<CoarseGridBasedData>>::GetInstance()->Register("gravity", [](const json& config) {
		return new SampledObjectGravity<CoarseGridBasedData>(config);
	});
	const bool fixture_force_registered = Factory<ExternalForce<CoarseGridBasedData>>::GetInstance()->Register("fixture-force", [](const json& config) {
		return new SampledObjecFixtureForce<CoarseGridBasedData>(config);
	});
}