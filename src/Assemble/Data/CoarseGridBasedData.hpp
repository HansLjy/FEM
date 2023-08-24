#pragma once
#include "MassSpringData.hpp"

struct CoarseGridBasedData : public MassSpringData {
	
	void Refresh();

	struct {
		int _num_points;
		VectorXd _x;
		MatrixXi _topo;
	} _proxy;


};

