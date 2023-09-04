#pragma once

#include "EigenAll.h"

class NullMassModel {
public:
	template<class Data> static void GetMass(const Data* obj, COO& coo, int x_offset, int y_offset) {}
	template<class Data> static double GetTotalMass(const Data* obj) {return 0;}

	template<class Data> static Vector3d GetUnnormalizedMassCenter(const Data* obj) {return Vector3d::Zero();}
	template<class Data> static Matrix3d GetInertialTensor(const Data* obj) {return Matrix3d::Zero();}
	template<class Data> static VectorXd GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) {return VectorXd(0);}
	// The inertial force in **local coordinate**
	template<class Data> static VectorXd GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration) {return VectorXd(0);}
};

template <class ProxyMassModel>
class ReducedMassModel {
public:
	template<class Data> static void GetMass(const Data* obj, COO& coo, int x_offset, int y_offset);
	template<class Data> static double GetTotalMass(const Data* obj);

	template<class Data> static Vector3d GetUnnormalizedMassCenter(const Data* obj);
	template<class Data> static Matrix3d GetInertialTensor(const Data* obj);
	template<class Data> static VectorXd GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation);
	// The inertial force in **local coordinate**
	template<class Data> static VectorXd GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration);
};

class SampledObjectMassModel {
public:
	template<class Data> static void GetMass(const Data* obj, COO& coo, int x_offset, int y_offset);
	template<class Data> static double GetTotalMass(const Data* obj);

	template<class Data> static Vector3d GetUnnormalizedMassCenter(const Data* obj);
	template<class Data> static Matrix3d GetInertialTensor(const Data* obj);
	template<class Data> static VectorXd GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation);
	// The inertial force in **local coordinate**
	template<class Data> static VectorXd GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration);
};


template<class ProxyMassModel>
template<class Data> void ReducedMassModel<ProxyMassModel>::GetMass(const Data* obj, COO& coo, int x_offset, int y_offset) {
	// TODO: make this faster
	COO coo_full;
	ProxyMassModel::GetMass(obj->_proxy, coo_full, 0, 0);
    SparseMatrixXd mass(obj->_proxy->_dof, obj->_proxy->_dof);
    mass.setFromTriplets(coo_full.begin(), coo_full.end());
    SparseMatrixXd mass_reduced = obj->_base.transpose() * mass * obj->_base;
	SparseToCOO(mass_reduced, coo, x_offset, y_offset);
}

template<class ProxyMassModel>
template<class Data> double ReducedMassModel<ProxyMassModel>::GetTotalMass(const Data* obj) {
	return ProxyMassModel::GetTotalMass(obj->_proxy);
}

template<class ProxyMassModel>
template<class Data> Vector3d ReducedMassModel<ProxyMassModel>::GetUnnormalizedMassCenter(const Data* obj) {
	return ProxyMassModel::GetUnnormalizedMassCenter(obj->_proxy);
}

template<class ProxyMassModel>
template<class Data> Matrix3d ReducedMassModel<ProxyMassModel>::GetInertialTensor(const Data* obj) {
	return ProxyMassModel::GetInertialTensor(obj->_proxy);
}

template<class ProxyMassModel>
template<class Data> VectorXd ReducedMassModel<ProxyMassModel>::GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) {
	return obj->_base.transpose() * ProxyMassModel::GetInertialForce(obj->_proxy, v, a, omega, alpha, rotation);
}

template<class ProxyMassModel>
template<class Data> VectorXd ReducedMassModel<ProxyMassModel>::GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration) {
	return obj->_base.transpose() * ProxyMassModel::GetInertialForce(obj->_proxy, v, a, affine, affine_velocity, affine_acceleration);
}


template<class Data> void SampledObjectMassModel::GetMass(const Data *obj, COO &coo, int x_offset, int y_offset) {
	for (int i = 0, j = 0; i < obj->_num_points; i++, j += 3) {
		coo.push_back(Tripletd(x_offset + j, y_offset + j, obj->_mass(i)));
		coo.push_back(Tripletd(x_offset + j + 1, y_offset + j + 1, obj->_mass(i)));
		coo.push_back(Tripletd(x_offset + j + 2, y_offset + j + 2, obj->_mass(i)));
	}
}

template<class Data> double SampledObjectMassModel::GetTotalMass(const Data *obj) {
	return obj->_total_mass;
}

template<class Data> Vector3d SampledObjectMassModel::GetUnnormalizedMassCenter(const Data* obj) {
	Vector3d mass_center = Vector3d::Zero();
	for (int i = 0, i3 = 0; i < obj->_num_points; i++, i3 += 3) {
		mass_center += obj->_mass(i) * obj->_x.template segment<3>(i3);
	}
	return mass_center;
}

template<class Data> Matrix3d SampledObjectMassModel::GetInertialTensor(const Data* obj) {
	Matrix3d inertial_tensor = Matrix3d::Zero();
	for (int i = 0, i3 = 0; i < obj->_num_points; i++, i3 += 3) {
		inertial_tensor += obj->_mass(i) * (obj->_x.template segment<3>(i3)) * (obj->_x.template segment<3>(i3)).transpose();
	}
	return inertial_tensor;	
}

template<class Data> VectorXd SampledObjectMassModel::GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) {
	VectorXd inertial_force(obj->_dof);
	for (int i = 0, j = 0; i < obj->_num_points; i++, j += 3) {
		Vector3d x_object = rotation * obj->_x.template segment<3>(j);
        Vector3d v_object = rotation * obj->_v.template segment<3>(j);
        inertial_force.template segment<3>(j) = -obj->_mass(i) * rotation.transpose() * (
            alpha.cross(x_object)
            + omega.cross(omega.cross(x_object))
            + 2 * omega.cross(v_object)
            + a
        );
	}
	return inertial_force;
}

template<class Data> VectorXd SampledObjectMassModel::GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration) {
	VectorXd inertial_force(obj->_dof);
	for (int i = 0, i3 = 0; i < obj->_num_points; i++, i3 += 3) {
		Vector3d x = obj->_x.template segment<3>(i3), v = obj->_v.template segment<3>(i3);
		inertial_force.template segment<3>(i3) = - obj->_mass(i) * affine.inverse() * (
			affine_acceleration * x
			+ 2 * affine_velocity * v
			+ a
		);
	}
	return inertial_force;
}