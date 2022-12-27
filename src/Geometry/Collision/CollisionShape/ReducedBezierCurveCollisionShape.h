#pragma once

#include "CollisionShape.h"

class ReducedBezierCurveCollisionShape : public CollisionShape {
public:
	void ComputeCollisionShape(const Object &obj, const Ref<const VectorXd> &x) override;
};