#include "SampledData.hpp"
#include "FileIO.hpp"
#include "Pattern.h"

SampledObjectData::SampledObjectData(const VectorXd& x, const VectorXd& mass, int dimension, const MatrixXi& topo)
	: BasicData(x), _total_mass(mass.sum()), _mass(mass), _num_points(mass.size()) {
	switch (dimension) {
		case 3:
			_tet_topo = topo;
			TopoUtil::GenerateSurfaceTopo3D(topo, _face_topo, _edge_topo);
			break;
		case 2:
			_face_topo = topo;
			TopoUtil::GenerateSurfaceTopo2D(topo, _edge_topo);
			break;
		case 1:
			_edge_topo = topo;
			break;
		default:
			throw std::logic_error("Unsupported dimension!");
	}
	_num_edges = _edge_topo.rows();
	_num_faces = _face_topo.rows();
}