#pragma once
#include "EigenAll.h"
#include "JsonUtil.h"

// Spring mass force that fix an object to its rest shape
class FixtureEnergyModel {
public:
	explicit FixtureEnergyModel(const json& config) {}

	template<class Derived> double GetPotential(Derived* obj, const Ref<const VectorXd> &x) const;
	template<class Derived>	VectorXd GetPotentialGradient(Derived* obj, const Ref<const VectorXd> &x) const;
	template<class Derived>	void GetPotentialHessian(Derived* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const;
};

namespace FixtureEnergyFunction {
	double GetPotential (
		int num_points,
		double ret_stiffness,
		const VectorXd& x_rest,
		const Ref<const VectorXd> &x
	);

	VectorXd GetPotentialGradient (
		int num_points,
		double ret_stiffness,
		const VectorXd& x_rest,
		const Ref<const VectorXd> &x
	);

	void GetPotentialHessian (
		int num_points,
		double ret_stiffness,
		const Ref<const VectorXd> &x,
		COO &coo, int x_offset, int y_offset
	);
}

template<class Derived>
double FixtureEnergyModel::GetPotential(Derived* obj, const Ref<const VectorXd> &x) const {
	return FixtureEnergyFunction::GetPotential(
		obj->_num_points,
		obj->_ret_stiffness,
		obj->_x_rest,
		x
	);
}

template<class Derived>
VectorXd FixtureEnergyModel::GetPotentialGradient(Derived* obj, const Ref<const VectorXd> &x) const {
	return FixtureEnergyFunction::GetPotentialGradient(
		obj->_num_points,
		obj->_ret_stiffness,
		obj->_x_rest,
		x
	);
}

template<class Derived>
void FixtureEnergyModel::GetPotentialHessian(Derived* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
	return FixtureEnergyFunction::GetPotentialHessian(
		obj->_num_points,
		obj->_ret_stiffness,
		x, coo, x_offset, y_offset
	);
}
