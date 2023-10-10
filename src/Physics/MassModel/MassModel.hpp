#pragma once

#include "EigenAll.h"

class NullMassModel {
public:
	template<class Derived> static void GetMass(Derived* obj, COO& coo, int x_offset, int y_offset) {}
	template<class Derived> static double GetTotalMass(Derived* obj) {return 0;}

	template<class Derived> static Vector3d GetUnnormalizedMassCenter(Derived* obj) {return Vector3d::Zero();}
	template<class Derived> static Matrix3d GetInertialTensor(Derived* obj) {return Matrix3d::Zero();}
	template<class Derived> static VectorXd GetInertialForce(Derived* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) {return VectorXd(0);}
	// The inertial force in **local coordinate**
	template<class Derived> static VectorXd GetInertialForce(Derived* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration) {return VectorXd(0);}
};

template<class ProxyMassModel>
class ReducedMassModel {
public:
	template<class Derived> static void GetMass(Derived* obj, COO& coo, int x_offset, int y_offset);
	template<class Derived> static double GetTotalMass(Derived* obj);

	template<class Derived> static Vector3d GetUnnormalizedMassCenter(Derived* obj);
	template<class Derived> static Matrix3d GetInertialTensor(Derived* obj);
	template<class Derived> static VectorXd GetInertialForce(Derived* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation);
	// The inertial force in **local coordinate**
	template<class Derived> static VectorXd GetInertialForce(Derived* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration);
};

class SampledObjectMassModel {
public:
	template<class Derived> static void GetMass(Derived* obj, COO& coo, int x_offset, int y_offset);
	template<class Derived> static double GetTotalMass(Derived* obj);

	template<class Derived> static Vector3d GetUnnormalizedMassCenter(Derived* obj);
	template<class Derived> static Matrix3d GetInertialTensor(Derived* obj);
	template<class Derived> static VectorXd GetInertialForce(Derived* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation);
	// The inertial force in **local coordinate**
	template<class Derived> static VectorXd GetInertialForce(Derived* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration);
};

template <class ProxyMassModel>
template <class Derived>
void ReducedMassModel<ProxyMassModel>::GetMass(Derived* obj, COO& coo, int x_offset, int y_offset){
	// TODO: make this faster
	COO coo_full;
	ProxyMassModel::GetMass(obj->_proxy, coo_full, 0, 0);
    SparseMatrixXd mass(obj->_proxy->_dof, obj->_proxy->_dof);
    mass.setFromTriplets(coo_full.begin(), coo_full.end());
    SparseMatrixXd mass_reduced = obj->_base.transpose() * mass * obj->_base;
	SparseToCOO(mass_reduced, coo, x_offset, y_offset);
}

template <class ProxyMassModel>
template <class Derived>
double ReducedMassModel<ProxyMassModel>::GetTotalMass(Derived* obj) {
	return ProxyMassModel::GetTotalMass(obj->_proxy);
}

template <class ProxyMassModel>
template <class Derived>
Vector3d ReducedMassModel<ProxyMassModel>::GetUnnormalizedMassCenter(Derived* obj) {
	return ProxyMassModel::GetUnnormalizedMassCenter(obj->_proxy);
}

template <class ProxyMassModel>
template <class Derived>
Matrix3d ReducedMassModel<ProxyMassModel>::GetInertialTensor(Derived* obj) {
	return ProxyMassModel::GetInertialTensor(obj->_proxy);
}

template <class ProxyMassModel>
template <class Derived>
VectorXd ReducedMassModel<ProxyMassModel>::GetInertialForce(Derived* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation){
	return obj->_base.transpose() * ProxyMassModel::GetInertialForce(obj->_proxy, v, a, omega, alpha, rotation);
}

template <class ProxyMassModel>
template <class Derived>
VectorXd ReducedMassModel<ProxyMassModel>::GetInertialForce(Derived* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration) {
	return obj->_base.transpose() * ProxyMassModel::GetInertialForce(obj->_proxy, v, a, affine, affine_velocity, affine_acceleration);
}

namespace SampleObjectMassFunction {
	void GetMass(
		int num_points, const VectorXd& mass,
		COO& coo, int x_offset, int y_offset
	);
	Vector3d GetUnnormalizedMassCenter(int num_points, const VectorXd& mass, const VectorXd& x);
	Matrix3d GetInertialTensor(int num_points, const VectorXd& mass, const VectorXd& x);
	VectorXd GetRigidInertialForce(	
		int num_points, const VectorXd& mass,
		const VectorXd& x, const Vector3d &v, const Vector3d &a,
		const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation
	);

	// The inertial force in **local coordinate**
	VectorXd GetAffineInertialForce(
		int num_points, const VectorXd& mass,
		const VectorXd& x, const Vector3d& v, const Vector3d &a,
		const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration
	);
}

template<class Derived>
void SampledObjectMassModel::GetMass(Derived* obj, COO &coo, int x_offset, int y_offset) {
	SampleObjectMassFunction::GetMass(
		obj->_num_points,
		obj->_mass,
		coo, x_offset, y_offset
	);
}

template<class Derived>
double SampledObjectMassModel::GetTotalMass(Derived* obj) {
	return obj->_total_mass;
}

template<class Derived>
Vector3d SampledObjectMassModel::GetUnnormalizedMassCenter(Derived* obj) {
	return SampleObjectMassFunction::GetUnnormalizedMassCenter(obj->_num_points, obj->_mass, obj->_x);
}

template<class Derived>
Matrix3d SampledObjectMassModel::GetInertialTensor(Derived* obj) {
	return SampleObjectMassFunction::GetInertialTensor(obj->_num_points, obj->_mass, obj->_x);
}

template<class Derived>
VectorXd SampledObjectMassModel::GetInertialForce(Derived* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) {
	return SampleObjectMassFunction::GetRigidInertialForce(
		obj->_num_points, obj->_mass,
		obj->_x, v, a,
		omega, alpha, rotation
	);
}

template<class Derived>
VectorXd SampledObjectMassModel::GetInertialForce(Derived* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration) {
	return SampleObjectMassFunction::GetAffineInertialForce(
		obj->_num_points, obj->__mass,
		obj->_x, v, a,
		affine, affine_velocity, affine_acceleration
	);
}