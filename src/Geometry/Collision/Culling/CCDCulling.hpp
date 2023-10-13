#pragma once

#include "EigenAll.h"
#include "Collision/CollisionInfo.hpp"
#include "Collision/CollisionInterface.hpp"
#include "Collision/CollisionShape/CollisionShape.hpp"

class CCDCulling {
public:
	virtual void GetCCDSet(
		double d_hat,
		const std::vector<CollisionInterface>& objs,
		const std::vector<int>& offsets,
		std::vector<PrimitivePair>& ccd_set
	) = 0;

};