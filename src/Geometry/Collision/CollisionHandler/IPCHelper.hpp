#pragma once

#include "EigenAll.h"
#include "Collision/CollisionInfo.hpp"
#include "Collision/CollisionInterface.hpp"
#include "SpatialQuery/SpatialHashing.hpp"
#include "Object.hpp"
#include "Collision/CollisionUtil/CCD/CCD.h"
#include "Collision/CollisionUtil/Culling/CCDCulling.hpp"

class IPCEnergy : public InterfaceContainer<CollisionInterface> {
public:
	IPCEnergy(double d_hat, double kappa) : _d_hat(d_hat), _kappa(kappa) {}

	void BindObjects(
		const typename std::vector<Object>::const_iterator &begin,
		const typename std::vector<Object>::const_iterator &end
	) override;

	double GetBarrierEnergy(const std::vector<PrimitivePair>& constraint_set) const;
	VectorXd GetBarrierEnergyGradient(const std::vector<PrimitivePair>& constraint_set) const;
	void GetBarrierEnergyHessian(const std::vector<PrimitivePair>& constraint_set, COO &coo, int offset_x, int offset_y) const;

protected:
    double GetVFBarrierEnergy(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    double GetEEBarrierEnergy(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;
    Vector12d GetVFBarrierEnergyGradient(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    Vector12d GetEEBarrierEnergyGradient(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;
    Matrix12d GetVFBarrierEnergyHessian(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    Matrix12d GetEEBarrierEnergyHessian(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;

	int _total_dof;
	std::vector<int> _offsets;
    double _d_hat;
	double _kappa;
};

class MaxStepEstimator : public InterfaceContainer<CollisionInterface> {
public:
	MaxStepEstimator(double d_hat) : _d_hat(d_hat) {}

	void BindObjects(
		const typename std::vector<Object>::const_iterator &begin,
		const typename std::vector<Object>::const_iterator &end
	) override;

	double GetMaxStep(const VectorXd &p);

protected:
	std::vector<int> _offsets;

	double _d_hat;
	CCD* _ccd;	// TODO:
	CCDCulling* _culling;
	std::vector<PrimitivePair> _ccd_set;
};

