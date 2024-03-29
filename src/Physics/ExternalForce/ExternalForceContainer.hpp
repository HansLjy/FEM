#pragma once
#include "Pattern.h"
#include "ExternalForce/ExternalForce.hpp"

DEFINE_HAS_MEMBER(_frame_rotation)
DEFINE_HAS_MEMBER(_frame_x)

template<class Data>
class ExternalForceContainer {
public:
	static ExternalForceContainer CreateFromConfig(const json& config) {
		std::vector<ExternalForce<Data>*> external_forces;
		for (const auto& force_config : config) {
			external_forces.push_back(Factory<ExternalForce<Data>>::GetInstance()->GetProduct(force_config["type"], force_config));
		}
		return {external_forces};
	}
	
	ExternalForceContainer(const ExternalForceContainer<Data>& rhs) = delete;
	ExternalForceContainer(ExternalForceContainer<Data>&& rhs) = default;

	void AddExternalForce(const std::string &type, const json &config) {
		_external_forces.push_back(Factory<ExternalForce<Data>>::GetInstance()->GetProduct(type, config));
	}

	VectorXd GetExternalForce(const Data *data) const {
		VectorXd ext_force = VectorXd::Zero(data->_dof);

		Matrix3d rotation = Matrix3d::Identity();
		Vector3d translate = Vector3d::Zero();

		// TODO: consider frame
		// if (HAS_MEMBER(Data, _frame_rotation) && HAS_MEMBER(Data, _frame_x)) {
		// 	rotation = data->_frame_rotation;
		// 	translate = data->_frame_x;
		// }
		for (const auto& external_force: _external_forces) {
			ext_force += external_force->GetExternalForce(data, rotation, translate);
		}
		return ext_force;
	}

	VectorXd GetExternalForceWithFrame(const Data *data, const Matrix3d &rotation, const Vector3d &position) const {
		VectorXd ext_force = VectorXd::Zero(data->_dof);

		for (const auto& external_force : _external_forces) {
			ext_force += external_force->GetExternalForce(data, rotation, position);
		}
		return ext_force;
	}

	Vector3d GetTotalExternalForce(const Data *data, const Matrix3d &rotation, const Vector3d &position) const {
		Vector3d total_external_force = Vector3d::Zero();
		for (const auto& external_force : _external_forces) {
			total_external_force += external_force->GetTotalForce(data, rotation, position);
		}
		return total_external_force;
	}

	Matrix3d GetTotalExternalForceTorque(const Data *data, const Matrix3d &rotation, const Vector3d &position) const {
		Matrix3d total_external_force_torque = Matrix3d::Zero();
		for (const auto& external_force : _external_forces) {
			total_external_force_torque += external_force->GetTotalForceAffineTorque(data, rotation, position);
		}
		return total_external_force_torque;
	}
	
	~ExternalForceContainer() {
		for (const auto& external_force : _external_forces) {
			delete external_force;
		}
	}

    ExternalForceContainer<Data>& operator=(const ExternalForceContainer<Data>& rhs) = delete;

protected:
	ExternalForceContainer(const std::vector<ExternalForce<Data>*>& rhs) : _external_forces(rhs) {}
    std::vector<ExternalForce<Data>*> _external_forces;
};

template<class> class ReducedObjectData;
template<class ProxyData>
class ExternalForceContainer<ReducedObjectData<ProxyData>> {
public:
	void AddExternalForce(const std::string &type, const json &config) {
		_proxy_container.AddExternalForce(type, config);
	}

	VectorXd GetExternalForce(const ReducedObjectData<ProxyData> *data) const {
		Matrix3d rotation = Matrix3d::Identity();
		Vector3d translate = Vector3d::Zero();

		// TODO: consider frame
		// if (HAS_MEMBER(Data, _frame_rotation) && HAS_MEMBER(Data, _frame_x)) {
		// 	rotation = data->_frame_rotation;
		// 	translate = data->_frame_x;
		// }
		return data->_base.transpose() * _proxy_container.GetExternalForceWithFrame(data->_proxy, rotation, translate);
	}

	VectorXd GetExternalForceWithFrame(const ReducedObjectData<ProxyData> *data, const Matrix3d &rotation, const Vector3d &position) const {
		return data->_base.transpose() * _proxy_container.GetExternalForceWithFrame(data->_proxy, rotation, position);
	}

	Vector3d GetTotalExternalForce(const ReducedObjectData<ProxyData> *data, const Matrix3d &rotation, const Vector3d &position) const {
		return _proxy_container.GetTotalExternalForce(data->_proxy, rotation, position);
	}

	Matrix3d GetTotalExternalForceTorque(const ReducedObjectData<ProxyData> *data, const Matrix3d &rotation, const Vector3d &position) const {
		return _proxy_container.GetTotalExternalForceTorque(data->_proxy, rotation, position);
	}
	
	ExternalForceContainer() = default;
    ExternalForceContainer(const ExternalForceContainer<ReducedObjectData<ProxyData>>& rhs) = delete;
    ExternalForceContainer<ReducedObjectData<ProxyData>>& operator=(const ExternalForceContainer<ReducedObjectData<ProxyData>>& rhs) = delete;

	ExternalForceContainer<ProxyData> _proxy_container;
};

template<class Data, class Derived>
class ExternalForceContainerAdapter {
public:
	ExternalForceContainerAdapter(const ExternalForceContainerAdapter<Data, Derived>& rhs) = delete;
	ExternalForceContainerAdapter(ExternalForceContainer<Data>&& rhs) : _container(std::move(rhs)) {}

	VectorXd GetExternalForce() const {
		return _container.GetExternalForce(static_cast<const Derived*>(this));
	}

	VectorXd GetExternalForceWithFrame(const Matrix3d &rotation, const Vector3d &position) const {
		return _container.GetExternalForceWithFrame(static_cast<const Derived*>(this), rotation, position);
	}

	Vector3d GetTotalExternalForce(const Matrix3d &rotation, const Vector3d &position) const {
		return _container.GetTotalExternalForce(static_cast<const Derived*>(this), rotation, position);
	}

	Matrix3d GetTotalExternalForceTorque(const Matrix3d &rotation, const Vector3d &position) const {
		return _container.GetTotalExternalForceTorque(static_cast<const Derived*>(this), rotation, position);
	}

	ExternalForceContainer<Data> _container;
};

namespace ExternalForceRegistration {
	template <template<class> class ConcreteExternalForce, class Data>
	bool RegisterExternalForce(const std::string& name) {
		return Factory<ExternalForce<Data>>::GetInstance()->Register(name, [](const json& config) {
			return new ConcreteExternalForce<Data>(config);
		});
	}
}

