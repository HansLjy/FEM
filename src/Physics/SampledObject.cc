#include "SampledObject.hpp"
#include "GeometryUtil.h"

SampledObject::SampledObject(const VectorXd& x, const VectorXd& mass, int dimension, const MatrixXi& topo)
	: SampledObject(x, VectorXd::Zero(x.size()), mass, dimension, topo) {}

SampledObject::SampledObject(const VectorXd& x, const VectorXd& v, const VectorXd& mass, int dimension, const MatrixXi& topo)
	: ConcreteObject(x, v), _total_mass(mass.sum()), _mass(mass), _num_points(mass.size()) {
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

void SampledObject::GetMass(COO &coo, int x_offset, int y_offset) const {
	const int num_points = this->GetDOF() / 3;
	for (int i = 0, j = 0; i < num_points; i++, j += 3) {
		coo.push_back(Tripletd(x_offset + j, y_offset + j, _mass(i)));
		coo.push_back(Tripletd(x_offset + j + 1, y_offset + j + 1, _mass(i)));
		coo.push_back(Tripletd(x_offset + j + 2, y_offset + j + 2, _mass(i)));
	}
}

double SampledObject::GetTotalMass() const {
	return _total_mass;
}

Vector3d SampledObject::GetUnnormalizedMassCenter() const {
	Vector3d mass_center = Vector3d::Zero();
	for (int i = 0, i3 = 0; i < _num_points; i++, i3 += 3) {
		mass_center += _mass(i) * this->_x.segment<3>(i3);
	}
	return mass_center;
}

Matrix3d SampledObject::GetInertialTensor() const {
	Matrix3d inertial_tensor = Matrix3d::Zero();
	for (int i = 0, i3 = 0; i < _num_points; i++, i3 += 3) {
		inertial_tensor += _mass(i) * (this->_x.segment<3>(i3)) * (this->_x.segment<3>(i3)).transpose();
	}
	return inertial_tensor;
}

VectorXd SampledObject::GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const {
	VectorXd inertial_force(this->_dof);

	for (int i = 0, j = 0; i < _num_points; i++, j += 3) {
		Vector3d x_object = rotation * this->_x.segment<3>(j);
        Vector3d v_object = rotation * this->_v.segment<3>(j);
        inertial_force.segment<3>(j) = -_mass(i) * rotation.transpose() * (
            alpha.cross(x_object)
            + omega.cross(omega.cross(x_object))
            + 2 * omega.cross(v_object)
            + a
        );
	}
	return inertial_force;
}

VectorXd SampledObject::GetInertialForce(const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d &affine_velocity, const Matrix3d &affine_acceleration) const {
	VectorXd inertial_force(this->_dof);
	for (int i = 0, i3 = 0; i < _num_points; i++, i3 += 3) {
		Vector3d x = this->_x.segment<3>(i3), v = this->_v.segment<3>(i3);
		inertial_force.segment<3>(i3) = - _mass(i) * affine.inverse() * (
			affine_acceleration * x
			+ 2 * affine_velocity * v
			+ a
		);
	}
	return inertial_force;
}
