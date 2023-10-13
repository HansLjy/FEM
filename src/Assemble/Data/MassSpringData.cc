#include "MassSpringData.hpp"
#include "TopoUtil.hpp"
#include "InitUtils.hpp"
#include "FileIO.hpp"


MassSpringData::MassSpringData(
	const VectorXd& x_rest,
	const MatrixXi& topo,
	const VectorXd& mass,
	double stiffness) :
	SampledObjectData(x_rest, mass, 2, topo),
	_x_rest(x_rest), _stiffness(stiffness) {
	_rest_length.resize(_num_edges);
	for (int i = 0; i < _num_edges; i++) {
		_rest_length(i) = (x_rest.segment<3>(topo(i, 0) * 3) - x_rest.segment<3>(topo(i, 1) * 3)).norm();
	}
}
