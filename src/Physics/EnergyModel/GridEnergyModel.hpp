#pragma once
#include "EigenAll.h"
#include "JsonUtil.h"

class GridEnergyModel {
public:
	GridEnergyModel() = default;
	explicit GridEnergyModel(const json& config) {}

	template<class Data> void Initialize(const Data* data) {}
	template<class Data> double GetPotential(const Data* data, const Ref<const VectorXd> &x) const;
	double GetPotential(
		const Ref<const VectorXd> &x,
		int num_edges, int start_diag_edges, int num_points,
		double stiffness, double diag_stiffness, double ret_stiffness,
		const Ref<const MatrixXi>& edge_topo,
		const Ref<const VectorXd>& rest_length,
		const Ref<const VectorXd>& x_rest
	) const;
	template<class Data> VectorXd GetPotentialGradient(const Data* data, const Ref<const VectorXd> &x) const;
	VectorXd GetPotentialGradient(
		const Ref<const VectorXd>& x,
		int num_edges, int start_diag_edges, int num_points,
		double stiffness, double diag_stiffness, double ret_stiffness,
		const Ref<const MatrixXi>& edge_topo,
		const Ref<const VectorXd>& rest_length,
		const Ref<const VectorXd>& x_rest
	) const;
	template<class Data> void GetPotentialHessian(const Data* data, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const;

	void GetPotentialHessian(
		const Ref<const VectorXd> &x,
		int dof,
		int num_edges, int start_diag_edges, int num_points,
		double stiffness, double diag_stiffness, double ret_stiffness,
		const Ref<const MatrixXi>& edge_topo,
		const Ref<const VectorXd>& rest_length,
		const Ref<const VectorXd>& x_rest,
		COO &coo, int x_offset, int y_offset
	) const;
};

template<class Data>
double GridEnergyModel::GetPotential(const Data *data, const Ref<const VectorXd> &x) const {
	return GetPotential(
		x,
		data->_num_edges, data->_start_diag_edges, data->_num_points,
		data->_stiffness, data->_diag_stiffness, data->_ret_stiffness,
		data->_edge_topo, data->_rest_length, data->_x_rest
	);
}

template<class Data>
VectorXd GridEnergyModel::GetPotentialGradient(const Data* data, const Ref<const VectorXd> &x) const {
	return GetPotentialGradient(
		x,
		data->_num_edges, data->_start_diag_edges, data->_num_points,
		data->_stiffness, data->_diag_stiffness, data->_ret_stiffness,
		data->_edge_topo, data->_rest_length, data->_x_rest
	);
}

template<class Data>
void GridEnergyModel::GetPotentialHessian(const Data* data, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
	GridEnergyModel::GetPotentialHessian(
		x, data->_dof,
		data->_num_edges, data->_start_diag_edges, data->_num_points,
		data->_stiffness, data->_diag_stiffness, data->_ret_stiffness,
		data->_edge_topo, data->_rest_length, data->_x_rest,
		coo, x_offset, y_offset
	);
}