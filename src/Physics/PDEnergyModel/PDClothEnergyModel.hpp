#pragma once

#include "EigenAll.h"
#include "JsonUtil.h"
#include "TopoUtil.hpp"
#include "Parallel.hpp"

template<class Derived, ParallelType parallel_type>
class PDClothEnergyModel {
public:
    static PDClothEnergyModel CreateFromConfig(const json& config) {return {};}
    PDClothEnergyModel(const PDClothEnergyModel& rhs) = delete;
    PDClothEnergyModel(PDClothEnergyModel&& rhs) = default;
    void Initialize();
    double GetEnergy(const Ref<const VectorXd>& x) const;
    void GetGlobalMatrix(COO& coo, int x_offset, int y_offset) const;
    void LocalProject(const Ref<const VectorXd>& x, Ref<VectorXd> y) const;
};

namespace PDEnergyModelFunction {
    double GetPDSpringEnergy(
        const Ref<const VectorXd>& x,
        const Ref<const MatrixXi>& edge_topo,
        const std::vector<double>& rest_length,
        double stiffness
    );

    void GetPDSpringEnergyLHSMatrix(
        const Ref<const MatrixXi>& edge_topo,
        double stiffness,
        COO& coo, int x_offset, int y_offset
    );

    Vector3d SpringLocalProject(
        const Vector3d& x1, const Vector3d& x2, double rest_length
    );

	template<ParallelType type>
    void GetPDSpringEnergyRHSVector (
        const Ref<const VectorXd> &x,
        const Ref<const MatrixXi>& edge_topo,
        const std::vector<double>& rest_length,
        double stiffness,
        Ref<VectorXd> y
    );

	template<>
	void GetPDSpringEnergyRHSVector<ParallelType::kNone> (
        const Ref<const VectorXd> &x,
        const Ref<const MatrixXi>& edge_topo,
        const std::vector<double>& rest_length,
        double stiffness,
        Ref<VectorXd> y
    );

	template<>
	void GetPDSpringEnergyRHSVector<ParallelType::kCPU> (
        const Ref<const VectorXd> &x,
        const Ref<const MatrixXi>& edge_topo,
        const std::vector<double>& rest_length,
        double stiffness,
        Ref<VectorXd> y
    );

};

namespace PDEnergyModelFunction {
    void InitQuadraticBendingMatrix(
        const Ref<const VectorXd>& x,
        const Ref<const MatrixXi>& internal_edge_topo,
        double stiffness,
        SparseMatrixXd& Q
    );

    double GetPDQuadraticEnergy(
        const Ref<const VectorXd>& x,
        const SparseMatrixXd& Q
    );

    void GetPDQuadraticEnergyLHSMatrix(
        const SparseMatrixXd& Q,
        COO& coo, int x_offset, int y_offset
    );

    void GetPDQuadraticEnergyRHSVector(
        Ref<VectorXd> y
    );
}

#include <iostream>

template<class Derived, ParallelType parallel_type>
double PDClothEnergyModel<Derived, parallel_type>::GetEnergy(const Ref<const VectorXd> &x) const {
    auto data = static_cast<const Derived*>(this);
    return PDEnergyModelFunction::GetPDSpringEnergy(
               x, data->_edge_topo,
               data->_rest_length,
               data->_spring_stiffness
           ) +
           PDEnergyModelFunction::GetPDQuadraticEnergy(x, data->_quadratic_bending);

}

template<class Derived, ParallelType parallel_type>
void PDClothEnergyModel<Derived, parallel_type>::GetGlobalMatrix(COO& coo, int x_offset, int y_offset) const {
    auto data = static_cast<const Derived*>(this);
    PDEnergyModelFunction::GetPDSpringEnergyLHSMatrix(data->_edge_topo, data->_spring_stiffness, coo, x_offset, y_offset);
    PDEnergyModelFunction::GetPDQuadraticEnergyLHSMatrix(data->_quadratic_bending, coo, x_offset, y_offset);
}

template<class Derived, ParallelType parallel_type>
void PDClothEnergyModel<Derived, parallel_type>::LocalProject(const Ref<const VectorXd>& x, Ref<VectorXd> y) const {
    auto data = static_cast<const Derived*>(this);
    PDEnergyModelFunction::GetPDSpringEnergyRHSVector<parallel_type>(x, data->_edge_topo, data->_rest_length, data->_spring_stiffness, y);
    PDEnergyModelFunction::GetPDQuadraticEnergyRHSVector(y);
}

template<class Derived, ParallelType parallel_type>
void PDClothEnergyModel<Derived, parallel_type>::Initialize() {
    auto data = static_cast<Derived*>(this);
    MatrixXi internal_edge_topo;
    internal_edge_topo = TopoUtil::GetInternalEdge(data->_face_topo);
    PDEnergyModelFunction::InitQuadraticBendingMatrix(
        data->_x,
        internal_edge_topo,
        data->_bending_stiffness,
        data->_quadratic_bending
    );
}
