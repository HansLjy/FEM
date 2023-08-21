#include "SpringMassData.hpp"

void SpringMassData::AddTriangle(int id1, int id2, const Vector3d &position) {
	_edge_topo.row(_num_edges++) << id1, _num_points;
	_edge_topo.row(_num_edges++) << id2, _num_points;
	_face_topo.row(_num_faces++) << id1, id2, _num_points;
	_x.segment<3>(_dof) = position;
	_v.segment<3>(_dof).setZero();
	_dof += 3;
	const double mass_increment = _density * (_x_rest.segment<3>(id2 * 3) - _x_rest.segment<3>(id1 * 3)).cross(_x_rest.segment<3>(_num_points * 3) -_x_rest.segment<3>(id1 * 3)).norm();
	_total_mass += mass_increment;
	const double single_mass_increment = mass_increment / 3;
	_mass(id1) += single_mass_increment;
	_mass(id2) += single_mass_increment;
	_mass(_num_points) += single_mass_increment;
	_num_points++;
}

SpringMassData::SpringMassData(const VectorXd& x_rest, const MatrixXi& topo, double density, double stiffness, int initial_vertices_number)
	: SampledObjectData(x_rest, VectorXd::Zero(x_rest.size()), 2, topo) {
	
}

