#pragma once
#include "EigenAll.h"
#include "JsonUtil.h"

template<class EnergyModel, class Derived>
class EnergyModelAdapter {
public:
	EnergyModelAdapter(const json& config) : _model(config) {}
	double GetPotential(const Ref<const VectorXd> &x) const {
		return _model.GetPotential(static_cast<const Derived*>(this), x);
	}

	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const {
		return _model.GetPotentialGradient(static_cast<const Derived*>(this), x);
	}

	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
		_model.GetPotentialHessian(static_cast<const Derived*>(this), x, coo, x_offset, y_offset);
	}

private:
	EnergyModel _model;
};