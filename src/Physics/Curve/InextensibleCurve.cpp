//
// Created by hansljy on 10/7/22.
//

#include "InextensibleCurve.h"
#include "CurveShape/CurveShape.h"
#include <exception>

DEFINE_CLONE(Object, InextensibleCurve)

double InextensibleCurve::GetPotential() const {
    double potential = 0;
    Vector3d x_current = _x.block<3, 1>(3, 0);
    Vector3d e_prev = x_current - _x.block<3, 1>(0, 0);

    // (i - 1) -- e_prev --> (i, x_current) -- e_current --> (i + 1, x_next)
    for (int i = 1; i < _num_points - 1; i++) {
        Vector3d x_next = _x.block<3, 1>(3 * (i + 1), 0);
        Vector3d e_current = x_next - x_current;

        Vector3d kB = 2 * e_prev.cross(e_current) / (_rest_length(i - 1) * _rest_length(i) + e_prev.dot(e_current));
        potential += kB.dot(kB) / _voronoi_length(i) * _alpha(i);

        e_prev = e_current;
        x_current = x_next;
    }
    return potential;
}

VectorXd InextensibleCurve::GetPotentialGradient() const {
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

        const double coefficient = 2 * _alpha(i) / _rest_length(i);
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

void InextensibleCurve::GetPotentialHessian(COO &coo, int x_offset, int y_offset) const {
    throw std::logic_error("Currently, implicit Euler for elastic curve is unsupported");
}

int InextensibleCurve::GetConstraintSize() const {
    return _num_points - 1;
}

VectorXd InextensibleCurve::GetInnerConstraint(const VectorXd &x) const {
    VectorXd result(_num_points - 1);
    for (int i = 0, j = 0; i < _num_points - 1; i++, j += 3) {
        Vector3d e = x.segment<3>(j + 3) - x.segment<3>(j);
        result(i) = e.dot(e) - _rest_length(i) * _rest_length(i);
    }
    return result;
}

void InextensibleCurve::GetInnerConstraintGradient(const VectorXd &x, COO &coo, int x_offset, int y_offset) const {
    for (int i = 0, j = 0; i < _num_points - 1; i++, j += 3) {
        Vector3d e = x.segment<3>(j + 3) - x.segment<3>(j);
        for (int k = 0; k < 3; k++) {
            coo.push_back(Tripletd(x_offset + i, y_offset + j + k, -2 * e(k)));
            coo.push_back(Tripletd(x_offset + i, y_offset + j + k + 3, 2 * e(k)));
        }
    }
}
