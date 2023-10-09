#include "FixtureEnergyModel.hpp"

double FixtureEnergyFunction::GetPotential (
	int num_points,
	double ret_stiffness,
	const VectorXd& x_rest,
	const Ref<const VectorXd> &x
) {
	double energy = 0;
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		double delta_x = (x.segment<3>(i3) - x_rest.segment<3>(i3)).norm();
		energy += ret_stiffness * delta_x * delta_x / 2;
	}
	return energy;
}

VectorXd FixtureEnergyFunction::GetPotentialGradient (
	int num_points,
	double ret_stiffness,
	const VectorXd& x_rest,
	const Ref<const VectorXd> &x
) {
	VectorXd gradient = VectorXd::Zero(x.size());
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		Vector3d e = x.segment<3>(i3) - x_rest.segment<3>(i3);
		gradient.segment<3>(i3) += ret_stiffness * e;
	}
	return gradient;
}

void FixtureEnergyFunction::GetPotentialHessian (
	int num_points,
	double ret_stiffness,
	const Ref<const VectorXd> &x,
	COO &coo, int x_offset, int y_offset
) {
	int dof = num_points * 3;
	for (int i = 0; i < dof; i++) {
		coo.push_back(Tripletd(x_offset + i, y_offset + i, ret_stiffness));
	}
}