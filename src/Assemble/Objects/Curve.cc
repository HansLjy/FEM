#include "Object.hpp"
#include "Data/CurveData.hpp"
#include "Model/CurvePhysics.hpp"
#include "Render/RenderShape.hpp"
#include "Collision/CollisionShape/CollisionShape.h"

using Curve = ConcreteObject<CurveData, CurveData, CurvePhysics, SampledRenderShape, SampledCollisionShape>;
const bool curve_registered = Factory<Object>::GetInstance()->Register("curve",
	[](const json& config) {
		return new Curve(config);
	}
);

using BezierCurve = ConcreteObject<BezierCurveData, ReducedObjectData<CurveData>, ReducedPhysics<CurvePhysics>, ProxiedRenderShape<SampledRenderShape>, NullCollisionShape>;
const bool bezier_cloth_registered = Factory<Object>::GetInstance()->Register("bezier-curve",
	[](const json& config) {
		return new BezierCurve(config);
	}
);