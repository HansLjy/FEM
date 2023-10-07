#pragma once

#include "EigenAll.h"
#include "Collision/CollisionShape/CollisionShape.hpp"
#include "Collision/CollisionInfo.hpp"
#include <set>

class IPCHelper {
public:
	IPCHelper(const json& config);
	virtual ~IPCHelper() = default;
	
	template<class Object> void SetObjects(const typename std::vector<Object*>::const_iterator& begin, const typename std::vector<Object*>::const_iterator& end);

	typedef std::function<bool(const CollisionInfo&)> Judger;

	double GetBarrierEnergy() const;
	VectorXd GetBarrierEnergyGradient() const;
	void GetBarrierEnergyHessian(COO &coo, int offset_x, int offset_y) const;

    virtual void ComputeConstraintSet(const VectorXd &x) = 0;
    virtual double GetMaxStep(const VectorXd& p) = 0;
	
    std::vector<CollisionInfo> _constraint_set;

protected:
    double GetVFBarrierEnergy(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    double GetEEBarrierEnergy(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;
    Vector12d GetVFBarrierEnergyGradient(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    Vector12d GetEEBarrierEnergyGradient(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;
    Matrix12d GetVFBarrierEnergyHessian(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    Matrix12d GetEEBarrierEnergyHessian(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;

	int _dof;
	std::vector<int> _dofs;
	std::vector<int> _offsets;
	std::vector<CollisionShapeInterface*> _objs;

    double _d_hat;
	double _kappa;
};

template<class ConstraintSetGenerator, class MaxStepEstimator>
class ConcreteIPCHelper : public IPCHelper, public ConstraintSetGenerator, public MaxStepEstimator {
	ConcreteIPCHelper(const json& config) : IPCHelper(config), ConstraintSetGenerator(config["constraint-set-generator"]), MaxStepEstimator(config["max-step-generator"]) {}
	void ComputeConstraintSet(const VectorXd &x) override {
		ConstraintSetGenerator::ComputeConstraintSet(x, _objs, _d_hat, _constraint_set);
	}

	double GetMaxStep(const VectorXd &p) override {
		return MaxStepEstimator::GetMaxStep(p, _objs, _d_hat, _constraint_set);
	}
};

template<class Object>
void IPCHelper::SetObjects(const typename std::vector<Object*>::const_iterator& begin, const typename std::vector<Object*>::const_iterator& end) {
	_objs.clear();
	_dofs.clear();
	_dof = 0;
	for (auto itr = begin; itr != end; ++itr) {
		_objs.push_back(*itr);
		_dofs.push_back((*itr)->GetDOF());
		_offsets.push_back(_dof);
		_dof += (*itr)->GetDOF();
	}
}
