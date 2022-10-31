//
// Created by hansljy on 10/31/22.
//

#include "SampledObjectGravity.h"
#include "JsonUtil.h"
#include "Object.h"


DEFINE_CLONE(ExternalForce, SampledObjectGravity)

SampledObjectGravity::SampledObjectGravity(const nlohmann::json &config) : SampledObjectGravity(Json2Vec(config["g"])) {}

SampledObjectGravity::SampledObjectGravity(const Eigen::Vector3d &g) : _g(g) {}

double SampledObjectGravity::Energy(const Object &obj) const {
    const auto& sampled_obj = dynamic_cast<const SampledObject&>(obj);
    double energy = 0;
    const int num_points = sampled_obj._mass.size();
    for (int i = 0, j = 0; i < num_points; i++, j += 3) {
        energy -= _g.dot(sampled_obj._x.segment<3>(j)) * sampled_obj._mass(i);
    }
    return energy;
}

VectorXd SampledObjectGravity::EnergyGradient(const Object &obj) const {
    const auto& sampled_obj = dynamic_cast<const SampledObject&>(obj);
    VectorXd gradient;
    gradient.resize(sampled_obj.GetDOF());
    const int num_points = sampled_obj._mass.size();
    for (int i = 0, j = 0; i < num_points; i++, j += 3) {
        gradient.block<3, 1>(j, 0) = - sampled_obj._mass(i) * _g;
    }
    return gradient;
}

void SampledObjectGravity::EnergyHessian(const Object &obj, COO &coo, int x_offset, int y_offset) const {

}