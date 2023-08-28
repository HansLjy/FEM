#include "SampledData.hpp"
#include "FileIO.hpp"
#include "Pattern.h"

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

SampledObjectData::SampledObjectData(const std::string& filename, double density, int dimension) : BasicData(VectorXd(0)) {
	auto file_io = Factory<FileIO>::GetInstance()->GetProduct(filename.substr(filename.find_last_of(".") + 1));
	MatrixXi topo;
	file_io->LoadFromFile(MODEL_PATH + filename, _x, topo);
	_v = VectorXd::Zero(_x.size());
	_dof = _x.size();
	_num_points = _dof / 3;
	
	switch (dimension) {
		case 3:
			_tet_topo = topo;
			GenerateSurfaceTopo3D(topo, _face_topo, _edge_topo);
			_mass = GenerateMass3D(_x, density, topo);
			break;
		case 2:
			_face_topo = topo;
			GenerateSurfaceTopo2D(topo, _edge_topo);
			_mass = GenerateMass2D(_x, density, topo);
			break;
		case 1:
			_edge_topo = topo;
			break;
		default:
			throw std::logic_error("Unsupported dimension!");
	}
	_num_edges = _edge_topo.rows();
	_num_faces = _face_topo.rows();

	_total_mass = _mass.sum();
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

int GetVerticesCnt(const MatrixXi& topo, int IFN) {
	int max_index = -1;
	for (int i = 0; i < IFN; i++) {
		const auto& indices = topo.row(i);
		max_index = std::max(std::max(indices[0], indices[1]), std::max(indices[2], max_index));
	}
	return max_index + 1;
}

void DynamicSampledObjectData::SetIFN(int IFN) {
	_num_faces = IFN;
	_num_points = GetVerticesCnt(_face_topo, IFN);
	_dof = 3 * _num_points;
	_mass = GenerateMass2D(_x.head(_dof), _density, _face_topo.topRows(IFN));
	MatrixXi initial_edge_topo;
	GenerateSurfaceTopo2D(_face_topo, initial_edge_topo);
	_num_edges = initial_edge_topo.rows();
	_total_mass = _mass.sum();
}

DynamicSampledObjectData::DynamicSampledObjectData(const std::string& filename, double density, int IFN)
: SampledObjectData(filename, density, 2), _density(density) {
	SetIFN(IFN);
}
DynamicSampledObjectData::DynamicSampledObjectData(const VectorXd& x, double density, const MatrixXi& topo, int IFN)
: SampledObjectData(x, density, 2, topo), _density(density) {
	SetIFN(IFN);
}

void DynamicSampledObjectData::AddFace(int id1, int id2, const Vector3d &position) {
	_edge_topo.row(_num_edges++) << id1, _num_points;
	_edge_topo.row(_num_edges++) << id2, _num_points;
	_face_topo.row(_num_faces++) << id1, id2, _num_points;
	_x.segment<3>(_dof) = position;
	_v.segment<3>(_dof).setZero();
	_dof += 3;
	const double mass_increment = _density * (_x_rest.segment<3>(id2 * 3) - _x_rest.segment<3>(id1 * 3)).cross(_x_rest.segment<3>(_num_points * 3) -_x_rest.segment<3>(id1 * 3)).norm() / 2;
	_total_mass += mass_increment;
	const double single_mass_increment = mass_increment / 3;
	_mass(id1) += single_mass_increment;
	_mass(id2) += single_mass_increment;
	_mass(_num_points) += single_mass_increment;
	_num_points++;
}

void DynamicSampledObjectData::AddFace(int id1, int id2, int id3) {
	_edge_topo.row(_num_edges++) << id2, id3;
	_face_topo.row(_num_faces++) << id1, id2, id3;
	const double mass_increment = _density * (_x_rest.segment<3>(id2 * 3) - _x_rest.segment<3>(id1 * 3)).cross(_x_rest.segment<3>(id3 * 3) -_x_rest.segment<3>(id1 * 3)).norm() / 2;
	_total_mass += mass_increment;
	const double single_mass_increment = mass_increment / 3;
	_mass(id1) += single_mass_increment;
	_mass(id2) += single_mass_increment;
	_mass(id3) += single_mass_increment;
}