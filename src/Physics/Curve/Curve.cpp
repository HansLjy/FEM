//
// Created by hansljy on 10/7/22.
//

#include "Curve.h"
#include "CurveShape/CurveShape.h"
#include <exception>

DEFINE_CLONE(Object, Curve)

VectorXd Curve::GetX(const Vector3d &start, const Vector3d &end, int num_segments){
    VectorXd x(3 * (num_segments + 1));
    Vector3d delta = (end - start) / num_segments;
    Vector3d current = start;
    for (int i = 0, j = 0; i <= num_segments; i++, j += 3, current += delta) {
        x.segment<3>(j) = current;
    }
    return x;
}

Curve::Curve(double total_mass, double alpha, const Eigen::Vector3d &start, const Eigen::Vector3d &end,
             int num_segments) : Curve(total_mass, alpha, GetX(start, end, num_segments)) {}

#include "JsonUtil.h"

Curve::Curve(const nlohmann::json &config) :
    Curve(config["mass"], config["alpha"], Json2Vec(config["start"]), Json2Vec(config["end"]), config["segments"]){}

Curve::Curve(double total_mass, double alpha, const VectorXd &x)
    : ShapedObject(x, CurveShape()), _alpha(alpha), _num_points(x.size() / 3) {
    _x_rest = x;
    _mass.resize(_num_points);
    _mass.setConstant(total_mass / _num_points);
    _mass_sparse.resize(_num_points * 3);
    _mass_sparse.setConstant(total_mass / _num_points);
    _rest_length.resize(_num_points - 1);
    for (int i = 0, j = 0; i < _num_points - 1; i++, j += 3) {
        _rest_length(i) = (_x_rest.block<3, 1>(j + 3, 0) - _x_rest.block<3, 1>(j, 0)).norm();
    }
    _voronoi_length.resize(_num_points);
    for (int i = 1; i < _num_points - 1; i++) {
        _voronoi_length(i) = (_rest_length(i - 1) + _rest_length(i)) / 2;
    }
    _voronoi_length(0) = _rest_length(0) / 2;
    _voronoi_length(_num_points - 1) = _rest_length(_num_points - 2) / 2;
}

void Curve::GetMass(COO &coo, int x_offset, int y_offset) const {
    for (int i = 0, j = 0; i < _num_points; i++, j += 3) {
        coo.push_back(Tripletd(x_offset + j, y_offset + j, _mass(i)));
        coo.push_back(Tripletd(x_offset + j + 1, y_offset + j + 1, _mass(i)));
        coo.push_back(Tripletd(x_offset + j + 2, y_offset + j + 2, _mass(i)));
    }
}

double Curve::GetPotential() const {
    double potential = 0;
    Vector3d x_current = _x.block<3, 1>(3, 0);
    Vector3d e_prev = x_current - _x.block<3, 1>(0, 0);

    // (i - 1) -- e_prev --> (i, x_current) -- e_current --> (i + 1, x_next)
    for (int i = 1; i < _num_points - 1; i++) {
        Vector3d x_next = _x.block<3, 1>(3 * (i + 1), 0);
        Vector3d e_current = x_next - x_current;

        Vector3d kB = 2 * e_prev.cross(e_current) / (_rest_length(i - 1) * _rest_length(i) + e_prev.dot(e_current));
        potential += kB.dot(kB) / _voronoi_length(i);

        e_prev = e_current;
        x_current = x_next;
    }
    return potential * _alpha;
}

VectorXd Curve::GetPotentialGradient() const {
    VectorXd gradient;
    gradient.resizeLike(_x);
    gradient.setZero();

    Vector3d x_current = _x.block<3, 1>(3, 0);
    Vector3d e_prev = x_current - _x.block<3, 1>(0, 0);

    // (i - 1) -- e_prev --> (i, x_current) -- e_current --> (i + 1, x_next)
    for (int i = 1; i < _num_points - 1; i++) {
        Vector3d x_next = _x.block<3, 1>(3 * (i + 1), 0);
        Vector3d e_current = x_next - x_current;

        const double denominator = _rest_length(i - 1) * _rest_length(i) + e_prev.dot(e_current);
        Vector3d kB = 2 * e_prev.cross(e_current) / denominator;
        Matrix3d nabla_prev = (2 * HatMatrix(e_current) + kB * e_current.transpose()) / denominator;
        Matrix3d nabla_next = (2 * HatMatrix(e_prev) - kB * e_prev.transpose()) / denominator;
        Matrix3d nabla_current = - nabla_prev - nabla_next;

        const double coefficient = 2 * _alpha / _rest_length(i);
        Vector3d contribute_prev = coefficient * kB.transpose() * nabla_prev;
        Vector3d contribute_current = coefficient * kB.transpose() * nabla_current;
        Vector3d contribute_next = coefficient * kB.transpose() * nabla_next;

        gradient.block<3, 1>(3 * (i - 1), 0) += contribute_prev;
        gradient.block<3, 1>(3 * i, 0) += contribute_current;
        gradient.block<3, 1>(3 * (i + 1), 0) += contribute_next;

        e_prev = e_current;
        x_current = x_next;
    }
    return gradient;
}

void Curve::GetPotentialHessian(COO &coo, int x_offset, int y_offset) const {
    throw std::logic_error("Currently, implicit Euler for elastic curve is unsupported");
}

int Curve::GetConstraintSize() const {
    return _num_points - 1;
}

VectorXd Curve::GetInnerConstraint(const VectorXd &x) const {
    VectorXd result(_num_points - 1);
    for (int i = 0, j = 0; i < _num_points - 1; i++, j += 3) {
        Vector3d e = x.segment<3>(j + 3) - x.segment<3>(j);
        result(i) = e.dot(e) - _rest_length(i) * _rest_length(i);
    }
    return result;
}

void Curve::GetInnerConstraintGradient(const VectorXd &x, COO &coo, int x_offset, int y_offset) const {
    for (int i = 0, j = 0; i < _num_points - 1; i++, j += 3) {
        Vector3d e = x.segment<3>(j + 3) - x.segment<3>(j);
        for (int k = 0; k < 3; k++) {
            coo.push_back(Tripletd(x_offset + i, y_offset + j + k, -2 * e(k)));
            coo.push_back(Tripletd(x_offset + i, y_offset + j + k + 3, 2 * e(k)));
        }
    }
}
