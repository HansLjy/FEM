#pragma once

#include "JsonUtil.h"
#include "EigenAll.h"

class PDNullEnergyModel {
public:
    PDNullEnergyModel() = default;
    PDNullEnergyModel(const PDNullEnergyModel& rhs) = delete;
    PDNullEnergyModel(PDNullEnergyModel&& rhs) = default;
    double GetEnergy(const Ref<const VectorXd>& x) const {return 0;}
    void GetGlobalMatrix(COO& coo, int x_offset, int y_offset) const {}
    void LocalProject(const Ref<const VectorXd>& x, Ref<VectorXd> y) const {}
};

