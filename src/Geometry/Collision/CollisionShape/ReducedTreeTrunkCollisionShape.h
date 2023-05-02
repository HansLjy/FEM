#pragma once

#include "CollisionShape.h"

class ReducedTreeTrunk;

class ReducedTreeTrunkCollisionShape : public CollisionShape {
public:
	void Bind(const Object &obj) override;
	void ComputeCollisionShape(const Ref<const VectorXd> &x) override;
	Vector3d GetCollisionVertexVelocity(const Ref<const VectorXd> &v, int idx) const override;
	const BlockVector & GetVertexDerivative(int idx) const override;

protected:
	const ReducedTreeTrunk* _reduced_tree_trunk;
	std::vector<BlockVector> _vertex_derivatives;
};