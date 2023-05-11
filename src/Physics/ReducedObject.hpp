#pragma once
#include "Object.hpp"

class ReducedObject : public ConcreteObject {
public:
	ReducedObject(const VectorXd& x, ProxyObject* proxy, const SparseMatrixXd& base, const VectorXd& shift)
		: ConcreteObject(x), _proxy(proxy), _base(base), _shift(shift) {}

	void Initialize() override {
		_proxy->Initialize();
	}

	void SetCoordinate(const Ref<const VectorXd> &x) override {
		ConcreteObject::SetCoordinate(x);
		_proxy->SetCoordinate(_base * x + _shift);
	}
	void SetVelocity(const Ref<const VectorXd> &v) override {
		ConcreteObject::SetVelocity(v);
		_proxy->SetVelocity(_base * v);
	}

	double GetMaxVelocity(const Ref<const VectorXd> &v) const override {return _proxy->GetMaxVelocity(_base * v);}

	Vector3d GetUnnormalizedMassCenter() const override {
		return _proxy->GetUnnormalizedMassCenter();
	}

	Matrix3d GetInertialTensor() const override {
		return _proxy->GetInertialTensor();
	}

	void GetMass(COO &coo, int x_offset, int y_offset) const override;
	double GetTotalMass() const override {return _proxy->GetTotalMass();}

	double GetPotential(const Ref<const VectorXd> &x) const override {return _proxy->GetPotential(_base * x + _shift);}
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override { return _base.transpose() * _proxy->GetPotentialGradient(_base * x + _shift);}
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override;

    void AddExternalForce(const std::string& type, const json& config) override {_proxy->AddExternalForce(type, config);}
	VectorXd GetExternalForce() const override {return _base.transpose() * _proxy->GetExternalForce();}
	VectorXd GetExternalForceWithFrame(const Matrix3d &rotation, const Vector3d &position) const override {return _base.transpose() * _proxy->GetExternalForceWithFrame(rotation, position);}
	Vector3d GetTotalExternalForce(const Matrix3d &rotation, const Vector3d &position) const override {return _proxy->GetTotalExternalForce(rotation, position);};
	Matrix3d GetTotalExternalForceTorque(const Matrix3d &rotation, const Vector3d &position) const override {return _proxy->GetTotalExternalForceTorque(rotation, position);}

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override {return _base.transpose() * _proxy->GetInertialForce(v, a, omega, alpha, rotation);}
	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d &affine_velocity, const Matrix3d &affine_acceleration) const override {
		return _base.transpose() * _proxy->GetInertialForce(v, a, affine, affine_velocity, affine_acceleration);
	}

	virtual ~ReducedObject() {
		delete _proxy;
	}

protected:
	ProxyObject* _proxy;
	const SparseMatrixXd _base;
	const VectorXd _shift;
};

