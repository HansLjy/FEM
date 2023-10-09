#pragma once
#include "EigenAll.h"
#include "JsonUtil.h"

class GridEnergyModel {
public:
	GridEnergyModel() = default;
	explicit GridEnergyModel(const json& config) {}

	template<class Derived>	double GetPotential(Derived* obj, const Ref<const VectorXd> &x) const;
	template<class Derived> VectorXd GetPotentialGradient(Derived* obj, const Ref<const VectorXd> &x) const;
	template<class Derived> void GetPotentialHessian(Derived* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const;
};

namespace GridEnergyFunction {
	double GetPotential(
		const Ref<const VectorXd> &x,
		int num_edges, int start_diag_edges, int num_points,
		double stiffness, double diag_stiffness, double ret_stiffness,
		const Ref<const MatrixXi>& edge_topo,
		const Ref<const VectorXd>& rest_length,
		const Ref<const VectorXd>& x_rest
	);
	VectorXd GetPotentialGradient(
		const Ref<const VectorXd>& x,
		int num_edges, int start_diag_edges, int num_points,
		double stiffness, double diag_stiffness, double ret_stiffness,
		const Ref<const MatrixXi>& edge_topo,
		const Ref<const VectorXd>& rest_length,
		const Ref<const VectorXd>& x_rest
	);
	void GetPotentialHessian(
		const Ref<const VectorXd> &x,
		int dof,
		int num_edges, int start_diag_edges, int num_points,
		double stiffness, double diag_stiffness, double ret_stiffness,
		const Ref<const MatrixXi>& edge_topo,
		const Ref<const VectorXd>& rest_length,
		const Ref<const VectorXd>& x_rest,
		COO &coo, int x_offset, int y_offset
	);
}

template<class Derived>
double GridEnergyModel::GetPotential(Derived* obj, const Ref<const VectorXd> &x) const {
	return GridEnergyFunction::GetPotential(
		x,
		obj->_num_edges, obj->_start_diag_edges, obj->_num_points,
		obj->_stiffness, obj->_diag_stiffness, obj->_ret_stiffness,
		obj->_edge_topo, obj->_rest_length, obj->_x_rest
	);
}

template<class Derived>
VectorXd GridEnergyModel::GetPotentialGradient(Derived* obj, const Ref<const VectorXd> &x) const {
	return GridEnergyFunction::GetPotentialGradient(
		x,
		obj->_num_edges, obj->_start_diag_edges, obj->_num_points,
		obj->_stiffness, obj->_diag_stiffness, obj->_ret_stiffness,
		obj->_edge_topo, obj->_rest_length, obj->_x_rest
	);
}

template<class Derived>
void GridEnergyModel::GetPotentialHessian(Derived* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
	GridEnergyFunction::GetPotentialHessian(
		x, obj->_dof,
		obj->_num_edges, obj->_start_diag_edges, obj->_num_points,
		obj->_stiffness, obj->_diag_stiffness, obj->_ret_stiffness,
		obj->_edge_topo, obj->_rest_length, obj->_x_rest,
		coo, x_offset, y_offset
	);
}