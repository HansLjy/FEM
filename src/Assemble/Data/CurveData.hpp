#pragma once

#include "SampledData.hpp"
#include "JsonUtil.h"

struct CurveData : public SampledObjectData {
    double _stiffness;	// stiffness of the curve
    VectorXd _alpha;
    VectorXd _x_rest;           // rest shape of the curve
    VectorXd _rest_length;      // length of every edge in the rest shape
    VectorXd _voronoi_length;   // length under the government of one point

protected:
    CurveData(const VectorXd &x, const VectorXd& mass, const VectorXd& alpha, double stiffness);
};
