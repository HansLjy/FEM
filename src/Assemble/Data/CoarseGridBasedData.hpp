#pragma once
#include "MassSpringData.hpp"

struct CoarseGridBasedData : public MassSpringData {
public:
	CoarseGridBasedData(const VectorXd& proxy_x, const MatrixXi& proxy_topo, double proxy_density, int proxy_dimension, int proxy_IFN, double stiffness);
	void Update();
	SampledObjectData _proxy;
};

