#include "Cloth.hpp"

template<>
Factory<ExternalForce<ClothData>>* Factory<ExternalForce<ClothData>>::_the_factory = nullptr;

const bool cloth_registered = Factory<Object>::GetInstance()->Register("cloth",
	[](const json& config) {
		return new Cloth(config);
	}
);

template<>
Factory<ExternalForce<BezierClothData>>* Factory<ExternalForce<BezierClothData>>::_the_factory = nullptr;

const bool bezier_cloth_registered = Factory<Object>::GetInstance()->Register("bezier-cloth",
	[](const json& config) {
		return new BezierCloth(config);
	}
);