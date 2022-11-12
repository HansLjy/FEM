//
// Created by hansljy on 10/7/22.
//

#include "InextensibleCurve.h"
#include "CurveShape/CurveShape.h"
#include "JsonUtil.h"
#include <exception>

DEFINE_CLONE(Object, InextensibleCurve)

InextensibleCurve::InextensibleCurve(const nlohmann::json &config)
    : InextensibleCurve(config["density"], config["alpha-max"], config["alpha-min"],
                        Json2Vec(config["start"]), Json2Vec(config["end"]), config["segments"]){}

double InextensibleCurve::GetPotential(const Ref<const Eigen::VectorXd> &x) const {
    double potential = 0;
    Vector3d x_current = x.block<3, 1>(3, 0);
    Vector3d e_prev = x_current - x.block<3, 1>(0, 0);

    // (i - 1) -- e_prev --> (i, x_current) -- e_current --> (i + 1, x_next)
    for (int i = 1; i < _num_points - 1; i++) {
        Vector3d x_next = x.block<3, 1>(3 * (i + 1), 0);
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

#include "unsupported/Eigen/KroneckerProduct"

void InextensibleCurve::GetPotentialHessian(COO &coo, int x_offset, int y_offset) const {
    Vector3d x_current = _x.segment<3>(3);
    Vector3d e_prev = x_current - _x.segment<3>(0);

    // (i - 1) -- e_prev --> (i, x_current) -- e_current --> (i + 1, x_next)
    for (int i = 1, ii = 0; i < _num_points - 1; i++, ii += 3) {
        Vector3d x_next = _x.block<3, 1>(3 * (i + 1), 0);
        Vector3d e_current = x_next - x_current;

        const double denominator = _rest_length(i - 1) * _rest_length(i) + e_prev.dot(e_current);
        Vector3d kB = 2 * e_prev.cross(e_current) / denominator;

        const Matrix3d hat_e_current = HatMatrix(e_current);
        const Matrix3d hat_e_prev = HatMatrix(e_prev);

        Eigen::Vector<Matrix3d, 3> p_kB;

        p_kB(0) = (- 2 * hat_e_current + e_current * kB.transpose()) / denominator;
        p_kB(2) = (- 2 * hat_e_prev - e_prev * kB.transpose()) / denominator;
        p_kB(1) = - p_kB(2) - p_kB(0);

        Matrix9d hessian;
        hessian.setZero();

        static const Matrix3d I3 = Matrix3d::Identity();
        const auto& kB_kron_I3 = Eigen::KroneckerProduct(kB, I3);
        const auto& I3_kron_e_current = Eigen::KroneckerProduct(I3, e_current);
        const auto& I3_kron_e_prev = Eigen::KroneckerProduct(I3, e_prev);
        const auto& kB_kron_e_current = Eigen::KroneckerProduct(kB, e_current);
        const auto& kB_kron_e_prev = Eigen::KroneckerProduct(kB, e_prev);


        static const Matrix<double, 9, 3> vec_hat_matrix(GetVecHatMatrix());

        Matrix<Matrix<double, 3, 9>, 3, 3> p2_kB;

        p2_kB(0, 0) = ((
            -(I3_kron_e_current) * (-2 * HatMatrix(e_current) - kB * e_current.transpose())
            -(2 * vec_hat_matrix * e_current - kB_kron_e_current) * e_current.transpose()
        ) / (denominator * denominator)).transpose();

        p2_kB(2, 2) = ((
            - I3_kron_e_prev * (2 * hat_e_prev - kB * e_prev.transpose())
            + (2 * vec_hat_matrix * e_prev + kB_kron_e_prev) * e_prev.transpose()
        ) / (denominator * denominator)).transpose();

        p2_kB(0, 2) = (
            - 2 * vec_hat_matrix / denominator
            + I3_kron_e_current * (2 * hat_e_prev - kB * e_prev.transpose()) / (denominator * denominator)
            + kB_kron_I3 / denominator
            + (2 * vec_hat_matrix * e_current - kB_kron_e_current) * e_prev.transpose() / (denominator * denominator)
        ).transpose();

        p2_kB(2, 0) = (
            2 * vec_hat_matrix / denominator
            + (I3_kron_e_prev * (-2 * hat_e_current - kB * e_current.transpose())) / (denominator * denominator)
            + kB_kron_I3 / denominator
            - (2 * vec_hat_matrix * e_prev + kB_kron_e_prev) * e_current.transpose() / (denominator * denominator)
        ).transpose();

        p2_kB(1, 0) = - p2_kB(0, 0) - p2_kB(2, 0);
        p2_kB(1, 2) = - p2_kB(0, 2) - p2_kB(2, 2);
        p2_kB(0, 1) = - p2_kB(0, 0) - p2_kB(0, 2);
        p2_kB(2, 1) = - p2_kB(2, 2) - p2_kB(2, 0);
        p2_kB(1, 1) = p2_kB(0, 0) + p2_kB(2, 0) + p2_kB(0, 2) + p2_kB(2, 2);

        const double coeff = 2 * _alpha(i) / _voronoi_length(i);
        for (int j = 0, jj = 0; j < 3; j++, jj += 3) {
            for (int k = 0, kk = 0; k < 3; k++, kk += 3) {
                hessian.block<3, 3>(jj, kk) = coeff * (
                        p2_kB(j, k) * kB_kron_I3
                        + p_kB(k) * p_kB(j).transpose()
                ).transpose();
            }
        }

        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 9; k++) {
                if(hessian(j, k) != 0) {
                    coo.push_back(Tripletd(ii + j + x_offset, ii + k + y_offset, hessian(j, k)));
                }
            }
        }

        e_prev = e_current;
        x_current = x_next;
    }
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
