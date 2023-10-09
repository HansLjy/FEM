//
// Created by hansljy on 10/27/22.
//

#pragma once
#include "EigenAll.h"
#include "JsonUtil.h"


class ClothEnergyModel {
public:
	ClothEnergyModel(const json& config) {}
	template<class Derived> double GetPotential(Derived* obj, const Ref<const VectorXd> &x) const;
	template<class Derived> VectorXd GetPotentialGradient(Derived* obj, const Ref<const VectorXd> &x) const;
	template<class Derived> void GetPotentialHessian(Derived* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const;
};

namespace ClothEnergyFunction {
	double GetPotential(
		int num_faces,
		const MatrixXi& face_topo,
		int num_internal_edges,
		const MatrixXi& internal_edge,
		const VectorXd& internal_edge_k_bend,
		const VectorXd& internal_edge_length,
		const std::vector<Matrix2d>& inv,
		const std::vector<Matrix6d>& pFpx,
		double k_stretch, double k_shear,
		double stretch_u, double stretch_v,
		const VectorXd& area,
		const Ref<const VectorXd> &x
	);

	VectorXd GetPotentialGradient(
		int num_faces,
		const MatrixXi& face_topo,
		int num_internal_edges,
		const MatrixXi& internal_edge,
		const VectorXd& internal_edge_k_bend,
		const VectorXd& internal_edge_length,
		const std::vector<Matrix2d>& inv,
		const std::vector<Matrix6d>& pFpx,
		double k_stretch, double k_shear,
		double stretch_u, double stretch_v,
		const VectorXd& area,
		const Ref<const VectorXd> &x
	);

	void GetPotentialHessian(
		int num_faces,
		const MatrixXi& face_topo,
		int num_internal_edges,
		const MatrixXi& internal_edge,
		const VectorXd& internal_edge_k_bend,
		const VectorXd& internal_edge_length,
		const std::vector<Matrix2d>& inv,
		const std::vector<Matrix6d>& pFpx,
		double k_stretch, double k_shear,
		double stretch_u, double stretch_v,
		const VectorXd& area,
		const Ref<const VectorXd> &x,
		COO &coo, int x_offset, int y_offset
	);
	
}


template<class Derived>
double ClothEnergyModel::GetPotential(Derived* obj, const Ref<const VectorXd> &x) const {
	return ClothEnergyFunction::GetPotential(
		obj->_num_faces, obj->_face_topo,
		obj->_num_internal_edges, obj->_internal_edge,
		obj->_internal_edge_k_bend, obj->_internal_edge_length,
		obj->_inv, obj->_pFpx,
		obj->_k_stretch, obj->_k_shear,
		obj->_stretch_u, obj->_stretch_v,
		obj->_area, 
		x
	);
}

inline Eigen::Matrix<double, 9, 5>
CalculatePFPX(const Eigen::Vector3d &e0, const Eigen::Vector3d &e1, const Eigen::Vector3d &e2) {
    Eigen::Matrix<double, 9, 5> pFpX = Eigen::Matrix<double, 9, 5>::Zero();
    pFpX.col(0).segment<3>(0) = e1.cross(e2);
    pFpX.col(0).segment<3>(3) = e2.cross(e0);
    pFpX.col(0).segment<3>(6) = e0.cross(e1);
    pFpX.col(1).segment<3>(0) = 2 * e0;
    pFpX.col(2).segment<3>(0) = e1;
    pFpX.col(2).segment<3>(3) = e0;
    pFpX.col(3).segment<3>(0) = e2;
    pFpX.col(3).segment<3>(6) = e0;
    pFpX.col(4).segment<3>(3) = e2;
    pFpX.col(4).segment<3>(6) = e1;
    return pFpX;
}

template<class Derived>
VectorXd ClothEnergyModel::GetPotentialGradient(Derived* obj, const Ref<const VectorXd>& x) const {
	return ClothEnergyFunction::GetPotentialGradient(
		obj->_num_faces, obj->_face_topo,
		obj->_num_internal_edges, obj->_internal_edge,
		obj->_internal_edge_k_bend, obj->_internal_edge_length,
		obj->_inv, obj->_pFpx,
		obj->_k_stretch, obj->_k_shear,
		obj->_stretch_u, obj->_stretch_v,
		obj->_area, 
		x
	);
}

#include "Timer.h"

template<class Derived>
void ClothEnergyModel::GetPotentialHessian(Derived* obj, const Ref<const VectorXd>& x, COO &coo, int x_offset, int y_offset) const {
	return ClothEnergyFunction::GetPotentialHessian(
		obj->_num_faces, obj->_face_topo,
		obj->_num_internal_edges, obj->_internal_edge,
		obj->_internal_edge_k_bend, obj->_internal_edge_length,
		obj->_inv, obj->_pFpx,
		obj->_k_stretch, obj->_k_shear,
		obj->_stretch_u, obj->_stretch_v,
		obj->_area, 
		x, coo, x_offset, y_offset
	);
}