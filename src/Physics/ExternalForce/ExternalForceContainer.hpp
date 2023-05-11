#pragma once
#include "ExternalForceFactory.hpp"

template <class Object>
class ExternalForceContainer {
public:
    void AddExternalForce(const std::string& type, const json& config);
	VectorXd GetExternalForce(const Object* obj) const;
	VectorXd GetExternalForceWithFrame(const Object* obj, const Matrix3d &rotation, const Vector3d &position) const;
	Vector3d GetTotalExternalForce(const Object* obj, const Matrix3d &rotation, const Vector3d &position) const;
	Matrix3d GetTotalExternalForceTorque(const Object* obj, const Matrix3d &rotation, const Vector3d &position) const;

    virtual ~ExternalForceContainer();
	ExternalForceContainer() = default;
    ExternalForceContainer(const ExternalForceContainer<Object>& rhs) = delete;
    ExternalForceContainer<Object>& operator=(const ExternalForceContainer<Object>& rhs) = delete;

private:
    std::vector<ExternalForce<Object>*> _external_forces;
};

template <class Object>
void ExternalForceContainer<Object>::AddExternalForce(const std::string &type, const json &config) {
    _external_forces.push_back(ExternalForceFactory<Object>::Instance()->GetExternalForce(type, config));
}

template <class Object>
VectorXd ExternalForceContainer<Object>::GetExternalForce(const Object *obj) const {
    VectorXd ext_force = VectorXd::Zero(obj->_dof);

	Matrix3d rotation = obj->GetFrameRotation();
	Vector3d translate = obj->GetFrameX();
	for (const auto& external_force: _external_forces) {
		ext_force += external_force->GetExternalForce(obj, rotation, translate);
	}
	return ext_force;
}

template<class Object>
VectorXd ExternalForceContainer<Object>::GetExternalForceWithFrame(const Object *obj, const Matrix3d &rotation, const Vector3d &position) const {
    VectorXd ext_force = VectorXd::Zero(obj->_dof);

	for (const auto& external_force : _external_forces) {
		ext_force += external_force->GetExternalForce(obj, rotation, position);
	}
	return ext_force;
}

template<class Object>
Vector3d ExternalForceContainer<Object>::GetTotalExternalForce(const Object *obj, const Matrix3d &rotation, const Vector3d &position) const {
	Vector3d total_external_force = Vector3d::Zero();
	for (const auto& external_force : _external_forces) {
		total_external_force += external_force->GetTotalForce(obj, rotation, position);
	}
	return total_external_force;
}

template<class Object>
Matrix3d ExternalForceContainer<Object>::GetTotalExternalForceTorque(const Object *obj, const Matrix3d &rotation, const Vector3d &position) const {
	Matrix3d total_external_force_torque = Matrix3d::Zero();
	for (const auto& external_force : _external_forces) {
		total_external_force_torque += external_force->GetTotalForceAffineTorque(obj, rotation, position);
	}
	return total_external_force_torque;
}

template<class Object>
ExternalForceContainer<Object>::~ExternalForceContainer() {
	for (const auto& external_force : _external_forces) {
		delete external_force;
	}
}

#define PROXY_EXTERNAL_FORCES_TO_CONTAINER(Object) \
    void AddExternalForce(const std::string& type, const json& config) override {\
        ExternalForceContainer<Object>::AddExternalForce(type, config);\
    }\
	VectorXd GetExternalForce() const override {\
        return ExternalForceContainer<Object>::GetExternalForce(this);\
    }\
	VectorXd GetExternalForceWithFrame(const Matrix3d &rotation, const Vector3d &position) const override {\
        return ExternalForceContainer<Object>::GetExternalForceWithFrame(this, rotation, position);\
    }\
	Vector3d GetTotalExternalForce(const Matrix3d &rotation, const Vector3d &position) const override {\
        return ExternalForceContainer<Object>::GetTotalExternalForce(this, rotation, position);\
    }\
	Matrix3d GetTotalExternalForceTorque(const Matrix3d &rotation, const Vector3d &position) const override {\
        return ExternalForceContainer<Object>::GetTotalExternalForceTorque(this, rotation, position);\
    }\
	friend ExternalForceContainer<Object>;
