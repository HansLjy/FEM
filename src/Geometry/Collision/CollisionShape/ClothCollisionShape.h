#pragma once

#include "CollisionShape.h"

class ClothCollisionShape : public CollisionShape {
public:
	void ComputeCollisionShape(const Object &obj, const Ref<const VectorXd> &x) override;

	DERIVED_DECLARE_CLONE(CollisionShape)
};