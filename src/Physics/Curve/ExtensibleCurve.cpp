//
// Created by hansljy on 10/19/22.
//

#include "ExtensibleCurve.h"

DEFINE_CLONE(Object, ExtensibleCurve)

double ExtensibleCurve::GetPotential() const {
    double potential = 0;
    Vector3d x_current = _x.segment<3>(3);
    Vector3d e_prev = x_current - _x.segment<3>(0);

    potential += 0.5 * _k * (e_prev.norm() - _rest_length(0)) * (e_prev.norm() - _rest_length(0));

    // (i - 1) -- e_prev --> (i, x_current) -- e_current --> (i + 1, x_next)
    for (int i = 1; i < _num_points - 1; i++) {
        Vector3d x_next = _x.segment<3>(3 * (i + 1));
        Vector3d e_current = x_next - x_current;

        potential += 0.5 * _k * (e_current.norm() - _rest_length(i)) * (e_current.norm() - _rest_length(i));

        Vector3d kB = 2 * e_prev.cross(e_current) / (_rest_length(i - 1) * _rest_length(i) + e_prev.dot(e_current));
        potential += kB.dot(kB) / _voronoi_length(i) * _alpha;

        e_prev = e_current;
        x_current = x_next;
    }
    return potential;
}

VectorXd ExtensibleCurve::GetPotentialGradient() const {
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

    for (int i = 0, j = 0; i < _num_points - 1; i++, j += 3) {
        Vector3d e = _x.segment<3>(j + 3) - _x.segment<3>(j);
        Vector3d contribution = _k * (e.norm() - _rest_length(i)) * e.normalized();
        gradient.segment<3>(j) -= contribution;
        gradient.segment<3>(j + 3) += contribution;
    }

    return gradient;
}

void ExtensibleCurve::GetPotentialHessian(COO &coo, int x_offset, int y_offset) const {
    throw std::logic_error("Sorry, implicit Euler for extensible curve is not currently available");
}
