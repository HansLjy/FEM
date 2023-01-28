#pragma once

#include "DecomposedObject.h"
#include "ReducedTreeTrunk.h"

class AffineDecomposedTreeTrunk : public AffineDecomposedObject {
public:
	explicit AffineDecomposedTreeTrunk(const json& config);

protected:
	void CalculateRigidRotationInfos(const CalculateLevel& level, const Ref<const VectorXd>& x, std::vector<Matrix3d>& rotations, std::vector<MatrixXd>& rotation_gradient, std::vector<MatrixXd>& rotation_hessian) const override;

	ReducedTreeTrunk* _tree_trunk;
	std::vector<int> _children_position;
};