#include "Object.hpp"
#include "Data/GridData.hpp"
#include "Model/MassSpringModel.hpp"
#include "Model/GridPhysics.hpp"
#include "Render/GridShape.hpp"
#include "Collision/CollisionShape/CollisionShape.h"
#include "ExternalForce/SampledObjectGravity.hpp"
#include "ExternalForce/SampledObjectFixtureForce.hpp"

using CoarseGrid = ConcreteObject<GridData, GridData, GridPhysics, GridShape, NullCollisionShape>;

template<>
Factory<ExternalForce<GridData>>* Factory<ExternalForce<GridData>>::_the_factory = nullptr;

namespace {
	const bool coarse_grid_registered = Factory<Object>::GetInstance()->Register("grid", [](const json& config) {
		return new CoarseGrid(config);
	});
	const bool gravity_registered = Factory<ExternalForce<GridData>>::GetInstance()->Register("gravity", [](const json& config) {
		return new SampledObjectGravity<GridData>(config);
	});
	const bool fixture_force_registered = Factory<ExternalForce<GridData>>::GetInstance()->Register("fixture-force", [](const json& config) {
		return new SampledObjecFixtureForce<GridData>(config);
	});
}