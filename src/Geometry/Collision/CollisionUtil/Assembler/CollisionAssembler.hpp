#pragma once

#include "EigenAll.h"
#include "Collision/CollisionInterface.hpp"

class CollisionAssembler {
public:
	void ComputeCollisionVertex(
		const Ref<const VectorXd>& x,
		std::vector<CollisionInterface>& objs
	) const;

	void ComputeCollisionVertexVelocity(
		const Ref<const VectorXd>& v,
		std::vector<CollisionInterface>& objs
	) const;
};