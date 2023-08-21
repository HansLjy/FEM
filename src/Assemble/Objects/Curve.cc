#include "Object.hpp"
#include "Data/CurveData.hpp"
#include "Model/CurvePhysics.hpp"
#include "Render/CurveShape.hpp"
#include "Collision/CollisionShape/CollisionShape.h"

using Curve = ConcreteObject<CurveData, CurveData, CurvePhysics, CurveShape, SampledCollisionShape>;
const bool curve_registered = Factory<Object>::GetInstance()->Register("curve",
	[](const json& config) {
		return new Curve(config);
	}
);

using BezierCurve = ConcreteObject<BezierCurveData, ReducedObjectData<CurveData>, ReducedPhysics<CurvePhysics>, ProxiedRenderShape<CurveShape>, NullCollisionShape>;
const bool bezier_cloth_registered = Factory<Object>::GetInstance()->Register("bezier-curve",
	[](const json& config) {
		return new BezierCurve(config);
	}
);