//
// Created by hansljy on 10/11/22.
//

#include "CurveGravity.h"
#include "Curve/InextensibleCurve.h"
#include "JsonUtil.h"

DEFINE_CLONE(ExternalForce, CurveGravity)

CurveGravity::CurveGravity(const json &config) : CurveGravity(Json2Vec(config["g"])) {}

CurveGravity::CurveGravity(const Eigen::Vector3d &g) : _g(g) {}

double CurveGravity::Energy(const Object &obj) const {
    const auto& curve = dynamic_cast<const Curve&>(obj);
    double energy = 0;
    for (int i = 0, j = 0; i < curve._num_points; i++, j += 3) {
        energy -= _g.dot(curve._x.block<3, 1>(j, 0)) * curve._mass(i);
    }
    return energy;
}

VectorXd CurveGravity::EnergyGradient(const Object &obj) const {
    const auto& curve = dynamic_cast<const Curve&>(obj);
    VectorXd gradient;
    gradient.resize(curve.GetDOF());
    for (int i = 0, j = 0; i < curve._num_points; i++, j += 3) {
        gradient.block<3, 1>(j, 0) = - curve._mass(i) * _g;
    }
    return gradient;
}

void CurveGravity::EnergyHessian(const Object &obj, SparseMatrixXd &hessian) const {
    hessian.resize(obj.GetDOF(), obj.GetDOF());
    hessian.setZero();
}

void CurveGravity::EnergyHessian(const Object &obj, COO &coo, int x_offset, int y_offset) const {

}