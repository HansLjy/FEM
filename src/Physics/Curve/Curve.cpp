//
// Created by hansljy on 10/7/22.
//

#include "Curve.h"
#include "CurveShape/CurveShape.h"
#include <exception>

DEFINE_CLONE(Object, Curve)

Curve::Curve(const Vector3d &start, const Vector3d &end, int num_segments, double total_mass, double alpha)
    : _alpha(alpha), _num_points(num_segments + 1) {
    Vector3d delta = (end - start) / num_segments;
    _x.resize(3 * _num_points);
    _x_rest.resize(3 * _num_points);
    _v.resize(3 * _num_points);
    _v.setZero();
    Vector3d current = start;
    for (int i = 0, j = 0; i <= num_segments; i++, j += 3, current += delta) {
        _x.block<3, 1>(j, 0) = _x_rest.block<3, 1>(j, 0) = current;
    }
    _mass.resize(_num_points);
    _mass.setConstant(total_mass / _num_points);
    _mass_sparse.resize(3 * _num_points);
    _mass_sparse.setConstant(total_mass / _num_points);
    _rest_length.resize(num_segments);
    _rest_length.setConstant(delta.norm());
    _voronoi_length.resize(_num_points);
    _voronoi_length.setConstant(delta.norm());
    _voronoi_length(0) = _voronoi_length(_num_points - 1) = delta.norm() / 2;

    _shape = new CurveShape;
}

Curve::Curve(const nlohmann::json &config) {
    const auto& start_array = config["start"];
    const auto& end_array = config["end"];
    Vector3d start, end;
    for (int i = 0; i < 3; i++) {
        start(i) = start_array[i];
        end(i) = end_array[i];
    }
    const int num_segments = config["segments"];
    const double mass = config["mass"];
    const double alpha = config["alpha"];

    *this = Curve(start, end, num_segments, mass, alpha);
}

void Curve::GetMass(SparseMatrixXd &mass) const {
    mass = _mass_sparse.diagonal().sparseView();
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