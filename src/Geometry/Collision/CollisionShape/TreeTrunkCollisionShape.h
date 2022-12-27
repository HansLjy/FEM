#pragma once

#include "CollisionShape.h"

class TreeTrunkCollisionShape : public CollisionShape {
public:
	void ComputeCollisionShape(const Object &obj, const Ref<const VectorXd> &x) override;
};