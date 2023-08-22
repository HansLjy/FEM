#include "SpringMassData.hpp"
#include "GeometryUtil.h"

int GetMaxIndices(const MatrixXi& topo) {

}

void SpringMassData::AddTriangle(int id1, int id2, const Vector3d &position) {
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

void SpringMassData::AddTriangle(int id1, int id2, int id3) {
	_edge_topo.row(_num_edges++) << id2, id3;
	_face_topo.row(_num_faces++) << id1, id2, id3;
	const double mass_increment = _density * (_x_rest.segment<3>(id2 * 3) - _x_rest.segment<3>(id1 * 3)).cross(_x_rest.segment<3>(id3 * 3) -_x_rest.segment<3>(id1 * 3)).norm() / 2;
	_total_mass += mass_increment;
	const double single_mass_increment = mass_increment / 3;
	_mass(id1) += single_mass_increment;
	_mass(id2) += single_mass_increment;
	_mass(id3) += single_mass_increment;
}

SpringMassData::SpringMassData(const VectorXd& x_rest, const MatrixXi& topo, double density, double stiffness, int IFN)
: SampledObjectData (
	IFN < 0 ? x_rest : x_rest.head(GetMaxIndices(topo) * 3),
	density, 2,
	IFN < 0 ? topo : topo.topRows(IFN)
), _x_rest(x_rest), _density(density), _stiffness(stiffness) {
	// save space for future usage
	_face_topo.resize(topo.rows(), 3);
	MatrixXi edge_topo_tmp;
	GenerateSurfaceTopo2D(topo, edge_topo_tmp);

	const int num_total_edges = edge_topo_tmp.rows();
	_edge_topo.resize(edge_topo_tmp.rows(), 2);	
	_rest_length.resize(num_total_edges);
	for (int i = 0; i < num_total_edges; i++) {
		_rest_length(i) = (x_rest.segment<3>(edge_topo_tmp(i, 0)) - x_rest.segment<3>(edge_topo_tmp(i, 1))).norm();
	}
}
