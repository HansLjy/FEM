#include "SampledData.hpp"
#include "FileIO.hpp"
#include "Pattern.h"

SampledObjectData SampledObjectData::GetSampledObjectData(const VectorXd& x, const VectorXd& mass, const MatrixXi& topo, int codimension) {
	MatrixXi edge_topo(0, 2), face_topo(0, 3), tet_topo(0, 4);
	switch (codimension) {
		case 3:
			tet_topo = topo;
			TopoUtil::GenerateSurfaceTopo3D(topo, face_topo, edge_topo);
			break;
		case 2:
			face_topo = topo;
			TopoUtil::GenerateSurfaceTopo2D(topo, edge_topo);
			break;
		case 1:
			edge_topo = topo;
			break;
		default:
			throw std::logic_error("Unsupported dimension!");
	}
	const int num_points = x.size() / 3;
	const int num_edges = edge_topo.rows();
	const int num_faces = face_topo.rows();
	return {
		x, mass, tet_topo, face_topo, edge_topo
	};
}


SampledObjectData::SampledObjectData(
	const VectorXd& x,
	const VectorXd& mass,
	const MatrixXi& tet_topo,
	const MatrixXi& face_topo,
	const MatrixXi& edge_topo) : 
	BasicData(x),
	_num_points(x.size() / 3),
	_num_edges(edge_topo.rows()),
	_num_faces(face_topo.rows()),
	_edge_topo(edge_topo),
	_face_topo(face_topo),
	_tet_topo(tet_topo),
	_total_mass(mass.sum()),
	_mass(mass) {}