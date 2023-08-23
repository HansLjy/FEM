#include "Object.hpp"
#include "Data/MassSpringData.hpp"
#include "Model/MassSpringModel.hpp"
#include "Render/RenderShape.hpp"
#include "Collision/CollisionShape/CollisionShape.h"
#include "ExternalForce/SampledObjectFixForce.hpp"
#include "ExternalForce/SampledObjectGravity.hpp"

using MassSpring = ConcreteObject<MassSpringData, MassSpringData, MassSpringPhysics, SampledRenderShape, NullCollisionShape>;

template<>
Factory<ExternalForce<MassSpringData>>* Factory<ExternalForce<MassSpringData>>::_the_factory = nullptr;

namespace {
	const bool mass_spring_registered = Factory<Object>::GetInstance()->Register("mass-spring", [](const json& config) {
		return new MassSpring(config);
	});
	const bool gravity_registered = Factory<ExternalForce<MassSpringData>>::GetInstance()->Register("gravity", [](const json& config) {
		return new SampledObjectGravity<MassSpringData>(config);
	});
	const bool fix_force_registered = Factory<ExternalForce<MassSpringData>>::GetInstance()->Register("fixture-force", [](const json& config) {
		return new SampledObjecFixForce<MassSpringData>(config);
	});
}