#pragma once
#include "EigenAll.h"
#include "JsonUtil.h"

// Spring mass force that fix an object to its rest shape
class FixtureEnergyModel {
public:
	FixtureEnergyModel() = default;
	explicit FixtureEnergyModel(const json& config) {}

	template<class Data> void Initialize(const Data* data) {}
	template<class Data> double GetPotential(const Data* data, const Ref<const VectorXd> &x) const;
	template<class Data> VectorXd GetPotentialGradient(const Data* data, const Ref<const VectorXd> &x) const;
	template<class Data> void GetPotentialHessian(const Data* data, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const;
};

template<class Data>
double FixtureEnergyModel::GetPotential(const Data *data, const Ref<const VectorXd> &x) const {
	const int num_points = data->_num_points;
	const double ret_stiffness = data->_ret_stiffness;
	const VectorXd& x_rest = data->_x_rest;

	double energy = 0;
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		double delta_x = (x.segment<3>(i3) - x_rest.segment<3>(i3)).norm();
		energy += ret_stiffness * delta_x * delta_x / 2;
	}
	return energy;
}

template<class Data>
VectorXd FixtureEnergyModel::GetPotentialGradient(const Data* data, const Ref<const VectorXd> &x) const {
	const int dof = data->_dof;
	const int num_points = data->_num_points;
	const double ret_stiffness = data->_ret_stiffness;
	const VectorXd& x_rest = data->_x_rest;

	VectorXd gradient = VectorXd::Zero(dof);
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		Vector3d e = x.segment<3>(i3) - x_rest.segment<3>(i3);
		gradient.segment<3>(i3) += ret_stiffness * e;
	}
	return gradient;
}

template<class Data>
void FixtureEnergyModel::GetPotentialHessian(const Data* data, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
	const int dof = data->_dof;
	const double ret_stiffness = data->_ret_stiffness;
	
	for (int i = 0; i < dof; i++) {
		coo.push_back(Tripletd(x_offset + i, y_offset + i, ret_stiffness));
	}
}
