#pragma once
#include "EigenAll.h"
#include "JsonUtil.h"

class MassSpringEnergyModel {
public:
	MassSpringEnergyModel(const json& config) {}
	template<class Derived>	double GetPotential(Derived* obj, const Ref<const VectorXd> &x) const;
	template<class Derived>	VectorXd GetPotentialGradient(Derived* obj, const Ref<const VectorXd> &x) const;
	template<class Derived>	void GetPotentialHessian(Derived* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const;
};

namespace MassSpringEnergyFunction {
	double GetPotential(int num_edges, double stiffness, const MatrixXi& edge_topo, const VectorXd& rest_length, const Ref<const VectorXd> &x);
	VectorXd GetPotentialGradient(int num_edges, double stiffness, const MatrixXi& edge_topo, const VectorXd& rest_length, const Ref<const VectorXd> &x);
	void GetPotentialHessian(
		int num_edges, double stiffness, const MatrixXi& edge_topo, const VectorXd& rest_length,
		const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset
	);
}

template<class Derived>
double MassSpringEnergyModel::GetPotential(Derived* obj, const Ref<const VectorXd> &x) const {
	return MassSpringEnergyFunction::GetPotential(obj->_num_edges, obj->_stiffness, obj->_edge_topo, obj->_rest_length, x);
}

template<class Derived>
VectorXd MassSpringEnergyModel::GetPotentialGradient(Derived* obj, const Ref<const VectorXd> &x) const {
	return MassSpringEnergyFunction::GetPotentialGradient(obj->_num_edges, obj->_stiffness, obj->_edge_topo, obj->_rest_length, x);
}

template<class Derived>
void MassSpringEnergyModel::GetPotentialHessian(Derived* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
	MassSpringEnergyFunction::GetPotentialHessian(
		obj->_num_edges, obj->_stiffness,
		obj->_edge_topo, obj->_rest_length,
		x, coo, x_offset, y_offset
	);
}