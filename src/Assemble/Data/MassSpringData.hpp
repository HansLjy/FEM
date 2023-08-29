#pragma once

#include "SampledData.hpp"
#include "JsonUtil.h"

struct MassSpringData : public SampledObjectData {
	explicit MassSpringData(const json& config);
	MassSpringData(const std::string& filename, double density, double stiffness);
	MassSpringData(const VectorXd& x_rest, const MatrixXi& topo, double density, double stiffness);
	MassSpringData(const VectorXd& x_rest, const MatrixXi& topo, const VectorXd& mass, double stiffness);
	

	double _stiffness;	// stiffness matrix
	VectorXd _x_rest;
	VectorXd _rest_length;
};