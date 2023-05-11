#pragma once

#include "DecomposedObject.hpp"

class ReducedTreeTrunk;

class DecomposedTreeTrunk : public RigidDecomposedObject, public ProxyRenderShape, public NullCollisionShape {
public:
	DecomposedTreeTrunk(const json& config);
	void Distribute(const Ref<const VectorXd> &a) override;

	void Initialize() override {
		RigidDecomposedObject::Initialize();
		NullCollisionShape::Precompute(this);
	}

	PROXY_RENDER_SHAPE(ProxyRenderShape)
	PROXY_COLLISION_SHAPE(NullCollisionShape)

protected:
	ReducedTreeTrunk* _tree_trunk;
	std::vector<double> _children_positions;
};

class ReducedLeaf;

class DecomposedLeaf : public RigidDecomposedObject, public ProxyRenderShape, public NullCollisionShape {
public:
	DecomposedLeaf(const json& config);
	void Distribute(const Ref<const VectorXd> &a) override {}

	void Initialize() override {
		RigidDecomposedObject::Initialize();
		NullCollisionShape::Precompute(this);
	}

	PROXY_RENDER_SHAPE(ProxyRenderShape)
	PROXY_COLLISION_SHAPE(NullCollisionShape)
};