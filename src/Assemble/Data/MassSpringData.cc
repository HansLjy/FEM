#include "MassSpringData.hpp"
#include "GeometryUtil.h"

MassSpringData::MassSpringData(const json& config) : MassSpringData(Json2VecX(config["x-rest"]), Json2MatXi<3>(config["topo"]), static_cast<double>(config["density"]), config["stiffness"]) {}

MassSpringData::MassSpringData(const VectorXd& x_rest, const MatrixXi& topo, double density, double stiffness) : MassSpringData(x_rest, topo, GenerateMass2D(x_rest, density, topo), stiffness) {}

MassSpringData::MassSpringData(const VectorXd& x_rest, const MatrixXi& topo, const VectorXd& mass, double stiffness) : SampledObjectData(x_rest, mass, 2, topo), _x_rest(x_rest), _stiffness(stiffness) {
	_rest_length.resize(_num_edges);
	for (int i = 0; i < _num_edges; i++) {
		_rest_length(i) = (x_rest.segment<3>(topo(i, 0) * 3) - x_rest.segment<3>(topo(i, 1) * 3)).norm();
	}
}
