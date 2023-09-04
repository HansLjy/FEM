#include "Curve.hpp"

const bool curve_registered = Factory<Object>::GetInstance()->Register("curve",
	[](const json& config) {
		return new Curve(config);
	}
);

const bool bezier_cloth_registered = Factory<Object>::GetInstance()->Register("bezier-curve",
	[](const json& config) {
		return new BezierCurve(config);
	}
);

template<> Factory<ExternalForce<CurveData>>* Factory<ExternalForce<CurveData>>::_the_factory = nullptr;
template<> Factory<ExternalForce<BezierCurveData>>* Factory<ExternalForce<BezierCurveData>>::_the_factory = nullptr;