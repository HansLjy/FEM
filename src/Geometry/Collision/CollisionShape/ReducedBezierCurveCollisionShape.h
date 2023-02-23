#pragma once

#include "CollisionShape.h"

class ReducedBezierCurveCollisionShape : public CollisionShape {
public:
	void Bind(const Object &obj) override;
	void ComputeCollisionShape(const Ref<const VectorXd> &x) override;
	Vector3d GetCollisionVertexVelocity(const Ref<const VectorXd> &v, int idx) const override;
	const BlockVector & GetVertexDerivative(int idx) const override;
};