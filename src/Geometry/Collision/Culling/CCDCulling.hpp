#pragma once

#include "EigenAll.h"
#include "Collision/CollisionInfo.hpp"
#include "Collision/CollisionShape/CollisionShape.hpp"
#include "Collision/SpatialHashing/SpatialHashing.hpp"

struct PrimitivePair {
	CollisionType _type;
	int _obj_id1;
	int _obj_id2;
	int _primitive_id1;
	int _primitive_id2;
};

class CCDCulling {
public:
	virtual void GetCCDSet(
		double d_hat,
		const std::vector<CollisionShapeInterface*>& objs,
		const std::vector<int>& offsets,
		const std::vector<int>& dofs,
		std::vector<PrimitivePair>& ccd_set
	) = 0;

};