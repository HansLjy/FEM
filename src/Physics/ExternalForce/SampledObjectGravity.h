//
// Created by hansljy on 10/31/22.
//

#pragma once

#include "SampledObject.hpp"
#include "ExternalForce.hpp"
#include "JsonUtil.h"

template<class Object>
class SampledObjectGravity : public ExternalForce<Object> {
public:
    static const bool valid = std::is_base_of<SampledObject, Object>::value;

    SampledObjectGravity(const json& config);
    SampledObjectGravity(const Vector3d& g);

    VectorXd GetExternalForce(const Object* obj, const Matrix3d& rotation, const Vector3d& position) const override;
    Vector3d GetTotalForce(const Object* obj, const Matrix3d& rotation, const Vector3d& position) const override;
    Matrix3d GetTotalForceAffineTorque(const Object* obj, const Matrix3d& rotation, const Vector3d& position) const override;

protected:
    const Vector3d _g;
};

template<class Object>
SampledObjectGravity<Object>::SampledObjectGravity(const nlohmann::json &config) : SampledObjectGravity(Json2Vec(config["g"])) {}

template<class Object>
SampledObjectGravity<Object>::SampledObjectGravity(const Eigen::Vector3d &g) : _g(g) {}

template<class Object>
VectorXd SampledObjectGravity<Object>::GetExternalForce(const Object *obj, const Matrix3d &rotation, const Vector3d &position) const {
    VectorXd gravity(obj->_dof);
    const Vector3d g_local = rotation.transpose() * _g;
    for (int i = 0, j = 0; i < obj->_num_points; i++, j += 3) {
        gravity.segment<3>(j) = obj->_mass(i) * g_local;
    }
    return gravity;
}

template<class Object>
Vector3d SampledObjectGravity<Object>::GetTotalForce(const Object *obj, const Matrix3d &rotation, const Vector3d &position) const {
    const Vector3d g_local = rotation.transpose() * _g;
    return g_local * obj->_total_mass;
}

template<class Object>
Matrix3d SampledObjectGravity<Object>::GetTotalForceAffineTorque(const Object *obj, const Matrix3d &rotation, const Vector3d &position) const {
    const Vector3d g_local = rotation.transpose() * _g;
    Matrix3d total_torque = Matrix3d::Zero();
    for (int i = 0, i3 = 0; i < obj->_num_points; i++, i3 += 3) {
        total_torque += obj->_mass(i) * g_local * obj->_x.template segment<3>(i3).transpose();
    }
    return total_torque;
}
