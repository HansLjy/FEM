//
// Created by hansljy on 10/19/22.
//

#ifndef FEM_CURVE_H
#define FEM_CURVE_H

#include "EigenAll.h"
#include "JsonUtil.h"
#include "unsupported/Eigen/KroneckerProduct"

class CurveEnergyModel {
public:
	CurveEnergyModel(const json& config) {}
	template <class Derived> double GetPotential(Derived* obj, const Ref<const VectorXd> &x) const;
	template <class Derived> VectorXd GetPotentialGradient(Derived* obj, const Ref<const VectorXd> &x) const;
	template <class Derived> void GetPotentialHessian(Derived* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const;
};

namespace CurveEnergyFunction {
	double GetPotential(	
		int curve_num_points,
		double stiffness, const VectorXd& alpha,
		const VectorXd& rest_length, const VectorXd& voronoi_length,
		const Ref<const VectorXd> &x
	);
	
	VectorXd GetPotentialGradient(
		int curve_num_points,
		double stiffness, const VectorXd& alpha,
		const VectorXd& rest_length, const VectorXd& voronoi_length,
		const Ref<const VectorXd> &x
	);
	
	void GetPotentialHessian(
		int curve_num_points,
		double stiffness, const VectorXd& alpha,
		const VectorXd& rest_length, const VectorXd& voronoi_length,
		const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset
	);
}

template<class Derived>
double CurveEnergyModel::GetPotential(Derived* obj, const Ref<const VectorXd> &x) const {
	return CurveEnergyFunction::GetPotential(
		obj->_num_points,
		obj->_stiffness, obj->_alpha,
		obj->_rest_length, obj->_voronoi_length,
		x
	);
}

template<class Derived>
VectorXd CurveEnergyModel::GetPotentialGradient(Derived* obj, const Ref<const VectorXd> &x) const {
	return CurveEnergyFunction::GetPotentialGradient(
		obj->_num_points,
		obj->_stiffness, obj->_alpha,
		obj->_rest_length, obj->_voronoi_length,
		x
	);
}

template<class Derived>
void CurveEnergyModel::GetPotentialHessian(Derived* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
	return CurveEnergyFunction::GetPotentialHessian(
		obj->_num_points,
		obj->_stiffness, obj->_alpha,
		obj->_rest_length, obj->_voronoi_length,
		x, coo, x_offset, y_offset
	);
}

#endif //FEM_CURVE_H
