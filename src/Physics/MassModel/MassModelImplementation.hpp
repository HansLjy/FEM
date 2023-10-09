#pragma once

#include "EigenAll.h"

template<class MassModel, class Derived>
class MassModelImplementation {
public:
	void GetMass(COO& coo, int x_offset, int y_offset) {
		MassModel::GetMass(static_cast<Derived*>(this), coo, x_offset, y_offset);
	}

	double GetTotalMass() {
		return MassModel::GetTotalMass(static_cast<Derived*>(this));
	}

	Vector3d GetUnnormalizedMassCenter() {
		return MassModel::GetUnnormalizedMassCenter(static_cast<Derived*>(this));
	}

	Matrix3d GetInertialTensor() {
		return MassModel::GetInertialTensor(static_cast<Derived*>(this));
	}

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) {
		return MassModel::GetInertialForce(static_cast<Derived*>(this), v, a, omega, alpha, rotation);
	}
	// The inertial force in **local coordinate**
	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration) {
		return MassModel::GetInertialForce(static_cast<Derived*>(this), v, a, affine, affine_velocity, affine_acceleration);
	}
};