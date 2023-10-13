#pragma once

#include "EigenAll.h"

template<class PDEnergyModel, class Derived>
class PDEnergyModelAdapter : public PDEnergyModel {
public:
    using PDEnergyModel::PDEnergyModel;
    PDEnergyModelAdapter(PDEnergyModel&& rhs) : PDEnergyModel(std::move(rhs)) {}
    void Initialize();
    void GetGlobalMatrix(COO& coo, int x_offset, int y_offset) const;
    void LocalProject(const Ref<const VectorXd>& x, Ref<VectorXd> y) const;
};

template<class PDEnergyModel, class Derived>
void PDEnergyModelAdapter<PDEnergyModel, Derived>::Initialize() {
    PDEnergyModel::Initialize(static_cast<Derived*>(this));
}

template<class PDEnergyModel, class Derived>
void PDEnergyModelAdapter<PDEnergyModel, Derived>::GetGlobalMatrix(COO &coo, int x_offset, int y_offset) const {
    PDEnergyModel::GetGlobalMatrix(static_cast<const Derived*>(this), coo, x_offset, y_offset);
}

template<class PDEnergyModel, class Derived>
void PDEnergyModelAdapter<PDEnergyModel, Derived>::LocalProject(const Ref<const VectorXd> &x, Ref<VectorXd> y) const {
    PDEnergyModel::LocalProject(static_cast<const Derived*>(this), x, y);
}