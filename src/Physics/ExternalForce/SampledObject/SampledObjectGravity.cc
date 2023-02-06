//
// Created by hansljy on 10/31/22.
//

#include "SampledObjectGravity.h"
#include "JsonUtil.h"
#include "Object.h"


DEFINE_CLONE(ExternalForce, SampledObjectGravity)

SampledObjectGravity::SampledObjectGravity(const nlohmann::json &config) : SampledObjectGravity(Json2Vec(config["g"])) {}

SampledObjectGravity::SampledObjectGravity(const Eigen::Vector3d &g) : _g(g) {}

VectorXd SampledObjectGravity::GetExternalForce(const Object &obj, const Matrix3d &rotation, const Vector3d &position) const {
    const auto& sampled_obj = dynamic_cast<const SampledObject&>(obj);
    VectorXd gravity(sampled_obj.GetDOF());
    const Vector3d g_local = rotation.transpose() * _g;
    for (int i = 0, j = 0; i < sampled_obj._num_points; i++, j += 3) {
        gravity.segment<3>(j) = sampled_obj._mass(i) * g_local;
    }
    return gravity;
}

Vector3d SampledObjectGravity::GetTotalForce(const Object &obj, const Matrix3d &rotation, const Vector3d &position) const {
    const auto& sampled_obj = dynamic_cast<const SampledObject&>(obj);
    const Vector3d g_local = rotation.transpose() * _g;
    return g_local * sampled_obj._total_mass;
}

Matrix3d SampledObjectGravity::GetTotalForceAffineTorque(const Object &obj, const Matrix3d &rotation, const Vector3d &position) const {
    const auto& sampled_obj = dynamic_cast<const SampledObject&>(obj);
    const Vector3d g_local = rotation.transpose() * _g;
    Matrix3d total_torque = Matrix3d::Zero();
    for (int i = 0, i3 = 0; i < sampled_obj._num_points; i++, i3 += 3) {
        total_torque += sampled_obj._mass(i) * g_local * sampled_obj._x.segment<3>(i3).transpose();
    }
    return total_torque;
}