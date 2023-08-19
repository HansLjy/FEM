#include "Data.hpp"

SampledObjectData::SampledObjectData(const VectorXd& x, const VectorXd& mass, int dimension, const  MatrixXi& topo)
: SampledObjectData(x, VectorXd::Zero(x.size()), mass, dimension, topo) {}

SampledObjectData::SampledObjectData(const VectorXd& x, const VectorXd& v, const VectorXd& mass, int dimension, const MatrixXi& topo)
	: BasicData(x, v), _total_mass(mass.sum()), _mass(mass), _num_points(mass.size()) {
	switch (dimension) {
		case 3:
			_tet_topo = topo;
			GenerateSurfaceTopo3D(topo, _face_topo, _edge_topo);
			break;
		case 2:
			_face_topo = topo;
			GenerateSurfaceTopo2D(topo, _edge_topo);
			break;
		case 1:
			_edge_topo = topo;
			break;
		default:
			throw std::logic_error("Unsupported dimension!");
	}
}