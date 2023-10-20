#pragma once

#include "Collision/CollisionInterface.hpp"
#include "Collision/CollisionInfo.hpp"
#include "EigenAll.h"
#include "Collision/CollisionUtil/CCD/CCD.h"

class TOIEstimator {
public:
	explicit TOIEstimator(const json& config) : _ccd(Factory<CCD>::GetInstance()->GetProduct(config["CCD"]["type"], config["CCD"])) {}

	//<- <= 1 for global toi, > 1 for non-intersecting case
	double GetTOI(
		const std::vector<PrimitivePair>& constraint_set,
		const std::vector<CollisionInterface>& objs
	) const;

	double GetLocalTOIs(
		const std::vector<PrimitivePair>& constraint_set,
		const std::vector<CollisionInterface>& objs,
		std::vector<double>& local_tois
	) const;


protected:
	CCD* _ccd;
};