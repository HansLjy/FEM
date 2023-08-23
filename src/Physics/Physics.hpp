#pragma once

#include "EigenAll.h"
#include "JsonUtil.h"

class BasicPhysics {
public:
	template<class Data> void Initialize(Data* data) {}
	template<class Data> static int GetDOF(const Data* obj);
	template<class Data> static void GetCoordinate(const Data* obj, Ref<VectorXd> x);
	template<class Data> static void GetVelocity(const Data* obj, Ref<VectorXd> v);
	template<class Data> static void SetCoordinate(Data* obj, const Ref<const VectorXd>& x);
	template<class Data> static void SetVelocity(Data* obj, const Ref<const VectorXd>& v);
};

class SampledPhysics : public BasicPhysics {
public:
    template<class Data> static void GetMass(const Data* obj, COO& coo, int x_offset, int y_offset);

	template<class Data> static double GetTotalMass(const Data* obj);
	template<class Data> static Vector3d GetUnnormalizedMassCenter(const Data* obj);
	template<class Data> static Matrix3d GetInertialTensor(const Data* obj);
	template<class Data> static VectorXd GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation);
	// The inertial force in **local coordinate**
	template<class Data> static VectorXd GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration);
};

template <class ReducedModel>
class ReducedPhysics : public BasicPhysics {
public:
	explicit ReducedPhysics(const json& config) : _reduced_model(config) {}
	template<class Data> static void SetCoordinate(Data* obj, const Ref<const VectorXd>& x);
	template<class Data> static void SetVelocity(Data* obj, const Ref<const VectorXd>& v);

    template<class Data> static void GetMass(const Data* obj, COO& coo, int x_offset, int y_offset);
	template<class Data> static double GetTotalMass(const Data* obj);

	template<class Data> double GetPotential(const Data* obj, const Ref<const VectorXd> &x) const;
	template<class Data> VectorXd GetPotentialGradient(const Data* obj, const Ref<const VectorXd> &x) const;
	template<class Data> void GetPotentialHessian(const Data* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const;

	template<class Data> static Vector3d GetUnnormalizedMassCenter(const Data* obj);
	template<class Data> static Matrix3d GetInertialTensor(const Data* obj);
	template<class Data> static VectorXd GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation);
	// The inertial force in **local coordinate**
	template<class Data> static VectorXd GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration);
	

protected:
	ReducedModel _reduced_model;
};

class NullPhysics : public BasicPhysics {
	template<class Data> static void GetMass(const Data* obj, COO& coo, int x_offset, int y_offset) {}
	template<class Data> static double GetTotalMass(const Data* obj) {return 0;}

	template<class Data> double GetPotential(const Data* obj, const Ref<const VectorXd> &x) const {return 0;}
	template<class Data> VectorXd GetPotentialGradient(const Data* obj, const Ref<const VectorXd> &x) const {return VectorXd(0);}
	template<class Data> void GetPotentialHessian(const Data* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {}

	template<class Data> static Vector3d GetUnnormalizedMassCenter(const Data* obj) {return Vector3d::Zero();}
	template<class Data> static Matrix3d GetInertialTensor(const Data* obj) {return Matrix3d::Zero();}
	template<class Data> static VectorXd GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) {return VectorXd(0);}
	// The inertial force in **local coordinate**
	template<class Data> static VectorXd GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration) {return VectorXd(0);}
};

template<class Data> int BasicPhysics::GetDOF(const Data *obj) {
	return obj->_dof;
}

template<class Data> void BasicPhysics::GetCoordinate(const Data* obj, Ref<VectorXd> x) {
	x = obj->_x.head(obj->_dof);
}

template<class Data> void BasicPhysics::GetVelocity(const Data* obj, Ref<VectorXd> v) {
	v = obj->_v.head(obj->_dof);
}

template<class Data> void BasicPhysics::SetCoordinate(Data* obj, const Ref<const VectorXd>& x) {
	obj->_x.head(obj->_dof) = x;
}

template<class Data> void BasicPhysics::SetVelocity(Data *obj, const Ref<const VectorXd> &v) {
	obj->_v.head(obj->_dof) = v;
}

template<class Data> void SampledPhysics::GetMass(const Data *obj, COO &coo, int x_offset, int y_offset) {
	for (int i = 0, j = 0; i < obj->_num_points; i++, j += 3) {
		coo.push_back(Tripletd(x_offset + j, y_offset + j, obj->_mass(i)));
		coo.push_back(Tripletd(x_offset + j + 1, y_offset + j + 1, obj->_mass(i)));
		coo.push_back(Tripletd(x_offset + j + 2, y_offset + j + 2, obj->_mass(i)));
	}
}

template<class Data> double SampledPhysics::GetTotalMass(const Data *obj) {
	return obj->_total_mass;
}

template<class Data> Vector3d SampledPhysics::GetUnnormalizedMassCenter(const Data* obj) {
	Vector3d mass_center = Vector3d::Zero();
	for (int i = 0, i3 = 0; i < obj->_num_points; i++, i3 += 3) {
		mass_center += obj->_mass(i) * obj->_x.template segment<3>(i3);
	}
	return mass_center;
}

template<class Data> Matrix3d SampledPhysics::GetInertialTensor(const Data* obj) {
	Matrix3d inertial_tensor = Matrix3d::Zero();
	for (int i = 0, i3 = 0; i < obj->_num_points; i++, i3 += 3) {
		inertial_tensor += obj->_mass(i) * (obj->_x.template segment<3>(i3)) * (obj->_x.template segment<3>(i3)).transpose();
	}
	return inertial_tensor;	
}

template<class Data> VectorXd SampledPhysics::GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) {
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

template<class Data> VectorXd SampledPhysics::GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration) {
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

template<class ReducedModel>
template<class Data> void ReducedPhysics<ReducedModel>::SetCoordinate(Data* obj, const Ref<const VectorXd>& x) {
	BasicPhysics::SetCoordinate(obj, x);
	ReducedModel::SetCoordinate(obj->_proxy, obj->_base * x + obj->_shift);
}

template<class ReducedModel>
template<class Data> void ReducedPhysics<ReducedModel>::SetVelocity(Data* obj, const Ref<const VectorXd>& v) {
	BasicPhysics::SetVelocity(obj, v);
	ReducedModel::SetVelocity(obj->_proxy, obj->_base * v);
}

template<class ReducedModel>
template<class Data> void ReducedPhysics<ReducedModel>::GetMass(const Data* obj, COO& coo, int x_offset, int y_offset) {
	// TODO: make this faster
	COO coo_full;
	ReducedModel::GetMass(obj->_proxy, coo_full, 0, 0);
    SparseMatrixXd mass(ReducedModel::GetDOF(obj->_proxy), ReducedModel::GetDOF(obj->_proxy));
    mass.setFromTriplets(coo_full.begin(), coo_full.end());
    SparseMatrixXd mass_reduced = obj->_base.transpose() * mass * obj->_base;
	SparseToCOO(mass_reduced, coo, x_offset, y_offset);
}

template<class ReducedModel>
template<class Data> double ReducedPhysics<ReducedModel>::GetTotalMass(const Data* obj) {
	return ReducedModel::GetTotalMass(obj->_proxy);
}

template<class ReducedModel>
template<class Data> double ReducedPhysics<ReducedModel>::GetPotential(const Data* obj, const Ref<const VectorXd> &x) const {
	return _reduced_model.GetPotential(obj->_proxy, obj->_base * x + obj->_shift);
}

template<class ReducedModel>
template<class Data> VectorXd ReducedPhysics<ReducedModel>::GetPotentialGradient(const Data* obj, const Ref<const VectorXd> &x) const {
	return obj->_base.transpose() * _reduced_model.GetPotentialGradient(obj->_proxy, obj->_base * x + obj->_shift);
}

template<class ReducedModel>
template<class Data> void ReducedPhysics<ReducedModel>::GetPotentialHessian(const Data* obj, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
	COO coo_full;
	_reduced_model.GetPotentialHessian(obj->_proxy, obj->_base * x + obj->_shift, coo_full, 0, 0);
    SparseMatrixXd proxy_hessian(ReducedModel::GetDOF(obj->_proxy), ReducedModel::GetDOF(obj->_proxy));
    proxy_hessian.setFromTriplets(coo_full.begin(), coo_full.end());
    SparseMatrixXd reduced_hessian = obj->_base.transpose() * proxy_hessian * obj->_base;
	SparseToCOO(reduced_hessian, coo, x_offset, y_offset);

}

template<class ReducedModel>
template<class Data> Vector3d ReducedPhysics<ReducedModel>::GetUnnormalizedMassCenter(const Data* obj) {
	return ReducedModel::GetUnnormalizedMassCenter(obj->_proxy);
}

template<class ReducedModel>
template<class Data> Matrix3d ReducedPhysics<ReducedModel>::GetInertialTensor(const Data* obj) {
	return ReducedModel::GetInertialTensor(obj->_proxy);
}

template<class ReducedModel>
template<class Data> VectorXd ReducedPhysics<ReducedModel>::GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) {
	return obj->_base.transpose() * ReducedModel::GetInertialForce(obj->_proxy, v, a, omega, alpha, rotation);
}

template<class ReducedModel>
template<class Data> VectorXd ReducedPhysics<ReducedModel>::GetInertialForce(const Data* obj, const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration) {
	return obj->_base.transpose() * ReducedModel::GetInertialForce(obj->_proxy, v, a, affine, affine_velocity, affine_acceleration);
}
