#include "Data.hpp"

VectorXd GenerateMass2D(const VectorXd& x, double density, const MatrixXi& topo) {
	int num_faces = topo.rows();
	VectorXd mass = VectorXd::Zero(x.size() / 3);
	for (int i = 0; i < num_faces; i++) {
		const auto& indices = topo.row(i);
		const int id1 = indices[0], id2 = indices[1], id3 = indices[2];

		const double mass_increment = density
			* (x.segment<3>(id2 * 3) - x.segment<3>(id1 * 3))
			  .cross(x.segment<3>(id3 * 3) - x.segment<3>(id1 * 3)).norm() / 2;
		const double single_mass_increment = mass_increment / 3;
		mass(id1) += single_mass_increment;
		mass(id2) += single_mass_increment;
		mass(id3) += single_mass_increment;
	}
	return mass;
}

VectorXd GenerateMass3D(const VectorXd& x, double density, const MatrixXi& topo) {
	int num_tets = topo.rows();
	VectorXd mass = VectorXd::Zero(x.size() / 3);
	for (int i = 0; i < num_tets; i++) {
		const auto& indices = topo.row(i);
		const int id1 = indices[0], id2 = indices[1], id3 = indices[2], id4 = indices[3];

		const double mass_increment = density
			* (x.segment<3>(id2 * 3) - x.segment<3>(id1 * 3))
			  .cross(x.segment<3>(id3 * 3) - x.segment<3>(id1 * 3))
			  .dot(x.segment<3>(id4 * 3) - x.segment<3>(id1 * 3));
		const double single_mass_increment = mass_increment / 4;
		mass(id1) += single_mass_increment;
		mass(id2) += single_mass_increment;
		mass(id3) += single_mass_increment;
		mass(id4) += single_mass_increment;
	}
	return mass;
}

SampledObjectData::SampledObjectData(const VectorXd& x, double density, int dimension, const MatrixXi& topo)
	: SampledObjectData(x, dimension == 2 ? GenerateMass2D(x, density, topo) : GenerateMass3D(x, density, topo), dimension, topo) {}

SampledObjectData::SampledObjectData(const VectorXd& x, const VectorXd& mass, int dimension, const MatrixXi& topo)
	: BasicData(x), _total_mass(mass.sum()), _mass(mass), _num_points(mass.size()) {
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
	_num_edges = _edge_topo.rows();
	_num_faces = _face_topo.rows();
}