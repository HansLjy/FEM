#pragma once

#include "Object.h"

class ReducedTreeTrunk;

class DecomposedTreeTrunk : public DecomposedObject {
public:
	DecomposedTreeTrunk(const json& config);
	void CalculateChildrenFrame(const Ref<const VectorXd> &a) override;

protected:
	SparseMatrixXd GetChildProjection(double distance) const;

	ReducedTreeTrunk* _tree_trunk;
	std::vector<double> _children_positions;
};

class ReducedLeaf;

class DecomposedLeaf : public DecomposedObject {
public:
	DecomposedLeaf(const json& config);
	void CalculateChildrenFrame(const Ref<const VectorXd> &a) override {}
};