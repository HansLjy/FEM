#pragma once

#include "CollisionShape.h"

class CurveCollisionShape : public CollisionShape {
public:
	void Bind(const Object &obj) override;
	void ComputeCollisionShape(const Ref<const VectorXd> &x) override;
};