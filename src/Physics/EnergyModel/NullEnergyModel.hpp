#pragma once

#include "EigenAll.h"
#include "JsonUtil.h"

class NullEnergyModel {
public:
	NullEnergyModel(const json& config) {}
	template<class Derived>	double GetPotential(Derived* obj, const Ref<const VectorXd> &x) const {
        return 0;
    }

	template<class Derived>	VectorXd GetPotentialGradient(Derived* obj, const Ref<const VectorXd> &x) const {
        return VectorXd::Zero(x.size());
    }

	template<class Derived>	void GetPotentialHessian(Derived* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {}
};
