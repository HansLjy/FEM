#pragma once

#include "EigenAll.h"
#include "JsonUtil.h"
#include "TopoUtil.hpp"

class PDClothEnergyModel {
public:
    static PDClothEnergyModel CreateFromConfig(const json& config) {return {};}
    PDClothEnergyModel(const PDClothEnergyModel& rhs) = delete;
    PDClothEnergyModel(PDClothEnergyModel&& rhs) = default;
    template<class Data> void Initialize(Data* data);
    template<class Data> void GetGlobalMatrix(const Data* data, COO& coo, int x_offset, int y_offset) const;
    template<class Data> void LocalProject(const Data* data, const Ref<const VectorXd>& x, Ref<VectorXd> y) const;
};

namespace PDEnergyModelFunction {
    void GetPDSpringEnergyLHSMatrix(
        const Ref<const MatrixXi>& edge_topo,
        double stiffness,
        COO& coo, int x_offset, int y_offset
    );

    Vector3d SpringLocalProject(
        const Vector3d& x1, const Vector3d& x2, double rest_length
    );

    void GetPDSpringEnergyRHSVector (
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

    void GetPDQuadraticEnergyLHSMatrix(
        const SparseMatrixXd& Q,
        COO& coo, int x_offset, int y_offset
    );

    void GetPDQuadraticEnergyRHSVector(
        Ref<VectorXd> y
    );
}

template<class Data>
void PDClothEnergyModel::GetGlobalMatrix(const Data* data, COO& coo, int x_offset, int y_offset) const {
    PDEnergyModelFunction::GetPDSpringEnergyLHSMatrix(data->_edge_topo, data->_spring_stiffness, coo, x_offset, y_offset);
    PDEnergyModelFunction::GetPDQuadraticEnergyLHSMatrix(data->_quadratic_bending, coo, x_offset, y_offset);
}

template<class Data>
void PDClothEnergyModel::LocalProject(const Data* data, const Ref<const VectorXd>& x, Ref<VectorXd> y) const {
    PDEnergyModelFunction::GetPDSpringEnergyRHSVector(x, data->_edge_topo, data->_rest_length, data->_spring_stiffness, y);
    PDEnergyModelFunction::GetPDQuadraticEnergyRHSVector(y);
}

template<class Data>
void PDClothEnergyModel::Initialize(Data* data) {
    MatrixXi internal_edge_topo;
    TopoUtil::GetInternalEdge(data->_face_topo, internal_edge_topo);
    PDEnergyModelFunction::InitQuadraticBendingMatrix(
        data->_x,
        internal_edge_topo,
        data->_bending_stiffness,
        data->_quadratic_bending
    );
}
