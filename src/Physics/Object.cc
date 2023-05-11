#include "Object.hpp"

ConcreteObject::ConcreteObject(const VectorXd& x)
	: ConcreteObject(x, VectorXd::Zero(x.size())) {}

ConcreteObject::ConcreteObject(const VectorXd& x, const VectorXd& v)
	: _dof(x.size()), _x(x), _v(v) {}

Vector3d ConcreteObject::GetFrameX() const {
	return Vector3d::Zero();
}

Matrix3d ConcreteObject::GetFrameRotation() const {
	return Matrix3d::Identity();
}

ObjectFactory* ObjectFactory::_the_factory = nullptr;