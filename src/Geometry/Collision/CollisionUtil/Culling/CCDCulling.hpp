#pragma once

#include "EigenAll.h"
#include "Collision/CollisionInfo.hpp"
#include "Collision/CollisionInterface.hpp"

class CCDCulling {
public:
	virtual void GetCCDSet(
		const std::vector<CollisionInterface>& objs,
		const std::vector<int>& offsets,
		std::vector<PrimitivePair>& ccd_set
	) = 0;

	virtual ~CCDCulling() = default;

};