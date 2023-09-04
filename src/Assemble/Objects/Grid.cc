#include "Grid.hpp"
#include "Pattern.h"

template<>
Factory<ExternalForce<GridData>>* Factory<ExternalForce<GridData>>::_the_factory = nullptr;

namespace {
	const bool coarse_grid_registered = Factory<Object>::GetInstance()->Register("grid", [](const json& config) {
		return new Grid(config);
	});
	const bool gravity_registered = Factory<ExternalForce<GridData>>::GetInstance()->Register("gravity", [](const json& config) {
		return new SampledObjectGravity<GridData>(config);
	});
	const bool fixture_force_registered = Factory<ExternalForce<GridData>>::GetInstance()->Register("fixture-force", [](const json& config) {
		return new SampledObjecFixtureForce<GridData>(config);
	});
}