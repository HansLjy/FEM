#pragma once

#include "JsonUtil.h"
#include "EigenAll.h"

class PDNullEnergyModel {
public:
    static PDNullEnergyModel CreateFromConfig(const json& config) {return {};}
    PDNullEnergyModel(const PDNullEnergyModel& rhs) = delete;
    PDNullEnergyModel(PDNullEnergyModel&& rhs) = default;
    template<class Data> void Initialize(Data* data) {}
    template<class Data> void GetGlobalMatrix(const Data* data, COO& coo, int x_offset, int y_offset) const {}
    template<class Data> void LocalProject(const Data* data, const Ref<const VectorXd>& x, Ref<VectorXd> y) const {}
};

