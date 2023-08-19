#include "Object.hpp"
#include "Data/ClothData.hpp"
#include "Model/ClothPhysics.hpp"
#include "Render/RenderShape.hpp"
#include "Collision/CollisionShape/CollisionShape.h"

using Cloth = ConcreteObject<ClothData, ClothData, ClothPhysics, SampledRenderShape, SampledCollisionShape>;
const bool cloth_registered = Factory<Object>::GetInstance()->Register("cloth",
	[](const json& config) {
		return new Cloth(config);
	}
);

using BezierCloth = ConcreteObject<BezierClothData, ReducedObjectData<ClothData>, ReducedPhysics<ClothPhysics>, ProxiedRenderShape<SampledRenderShape>, NullCollisionShape>;
const bool bezier_cloth_registered = Factory<Object>::GetInstance()->Register("bezier-cloth",
	[](const json& config) {
		return new BezierCloth(config);
	}
);