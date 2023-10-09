#include "MassModel.hpp"

void SampleObjectMassFunction::GetMass(int num_points, const VectorXd &mass, COO &coo, int x_offset, int y_offset) {
	for (int i = 0, j = 0; i < num_points; i++, j += 3) {
		coo.push_back(Tripletd(x_offset + j, y_offset + j, mass(i)));
		coo.push_back(Tripletd(x_offset + j + 1, y_offset + j + 1, mass(i)));
		coo.push_back(Tripletd(x_offset + j + 2, y_offset + j + 2, mass(i)));
	}
}

Vector3d SampleObjectMassFunction::GetUnnormalizedMassCenter(int num_points, const VectorXd& mass, const VectorXd& x) {
	Vector3d mass_center = Vector3d::Zero();
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		mass_center += mass(i) * x.segment<3>(i3);
	}
	return mass_center;
}

Matrix3d SampleObjectMassFunction::GetInertialTensor(int num_points, const VectorXd& mass, const VectorXd& x) {
	Matrix3d inertial_tensor = Matrix3d::Zero();
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		inertial_tensor += mass(i) * (x.segment<3>(i3)) * (x.segment<3>(i3)).transpose();
	}
	return inertial_tensor;	
}

VectorXd SampleObjectMassFunction::GetRigidInertialForce(
	int num_points, const VectorXd& mass,
	const VectorXd& x, const Vector3d &v, const Vector3d &a,
	const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) {
	const int dof = num_points * 3;
	VectorXd inertial_force(dof);
	for (int i = 0, j = 0; i < num_points; i++, j += 3) {
		Vector3d x_object = rotation * x.segment<3>(j);
        Vector3d v_object = rotation * v.segment<3>(j);
        inertial_force.template segment<3>(j) = -mass(i) * rotation.transpose() * (
            alpha.cross(x_object)
            + omega.cross(omega.cross(x_object))
            + 2 * omega.cross(v_object)
            + a
        );
	}
	return inertial_force;

}

VectorXd SampleObjectMassFunction::GetAffineInertialForce(
	int num_points, const VectorXd& mass,
	const VectorXd& x, const Vector3d& v, const Vector3d &a,
	const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration
) {
	const int dof = num_points * 3;
	VectorXd inertial_force(dof);
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		Vector3d x_local = x.segment<3>(i3), v_local = v.template segment<3>(i3);
		inertial_force.template segment<3>(i3) = - mass(i) * affine.inverse() * (
			affine_acceleration * x_local
			+ 2 * affine_velocity * v_local
			+ a
		);
	}
	return inertial_force;
}