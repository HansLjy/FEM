#pragma once

#include "SampledData.hpp"
#include "JsonUtil.h"

struct MassSpringData : public SampledObjectData {
public:
	MassSpringData(const MassSpringData& rhs) = delete;
	MassSpringData(MassSpringData&& rhs) = default;

	double _stiffness;	// stiffness matrix
	VectorXd _x_rest;
	VectorXd _rest_length;

protected:
	MassSpringData(const VectorXd& x_rest, const MatrixXi& topo, const VectorXd& mass, double stiffness);
};