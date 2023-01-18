#pragma once

#include "DecomposedObject.h"

class ReducedTreeTrunk;

class DecomposedTreeTrunk : public RigidDecomposedObject {
public:
	DecomposedTreeTrunk(const json& config);
	void CalculateChildrenFrame(const Ref<const VectorXd> &a) override;

protected:
	SparseMatrixXd GetChildProjection(double distance) const;

	ReducedTreeTrunk* _tree_trunk;
	std::vector<double> _children_positions;
};

class ReducedLeaf;

class DecomposedLeaf : public RigidDecomposedObject {
public:
	DecomposedLeaf(const json& config);
	void CalculateChildrenFrame(const Ref<const VectorXd> &a) override {}
};