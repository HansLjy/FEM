#pragma once

#include "IPCHelper.hpp"
#include "Collision/CCD/CCD.h"
#include "Collision/Culling/CCDCulling.hpp"

class NormalMaxStepEstimator {
public:
	NormalMaxStepEstimator(const json& config);
	double GetMaxStep(
		const VectorXd &p,
		double d_hat,
		const std::vector<CollisionShapeInterface*>& objs,
		const std::vector<int>& offsets,
		const std::vector<int>& dofs
	);

protected:
	CCD* _ccd;
	CCDCulling* _culling;
	std::vector<PrimitivePair> _ccd_set;
};
