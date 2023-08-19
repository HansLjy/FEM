//
// Created by hansljy on 10/27/22.
//

#pragma once
#include "Physics.hpp"

class ClothPhysics : public SampledPhysics {
public:
	ClothPhysics(const json& config) {}
	template<class Data> double GetPotential(const Data* data, const Ref<const VectorXd> &x) const;
	template<class Data> VectorXd GetPotentialGradient(const Data* data, const Ref<const VectorXd> &x) const;
	template<class Data> void GetPotentialHessian(const Data* data, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const;
};

template<class Data>
double ClothPhysics::GetPotential(const Data* data, const Ref<const VectorXd> &x) const {
	double energy = 0;
    for (int i = 0; i < data->_num_triangles; i++) {
        RowVector3i index = data->_face_topo.row(i);
        Matrix<double, 3, 2> F;
        F.col(0) = x.segment<3>(3 * index(1)) - x.segment<3>(3 * index(0));
        F.col(1) = x.segment<3>(3 * index(2)) - x.segment<3>(3 * index(0));
        F = F * data->_inv(i);
        Vector3d F1 = F.col(0);
        Vector3d F2 = F.col(1);
        const double C1 = (F1.norm() - data->_stretch_u) * data->_area(i);
        const double C2 = (F2.norm() - data->_stretch_v) * data->_area(i);
        const double C3 = data->_area(i) * F1.dot(F2);

        energy += 0.5 * data->_k_stretch * (C1 * C1 + C2 * C2) + 0.5 * data->_k_shear * C3 * C3;
    }

    for (int i = 0; i < data->_num_internal_edges; i++) {
        RowVector4i edge_index = data->_internal_edge.row(i);
        Vector3d xi = x.segment<3>(3 * edge_index(0));
        Vector3d e0 = x.segment<3>(3 * edge_index(1)) - xi;
        Vector3d e1 = x.segment<3>(3 * edge_index(2)) - xi;
        Vector3d e2 = x.segment<3>(3 * edge_index(3)) - xi;

        const double X = data->_internal_edge_length(i) * (
                e0.dot(e2) * e0.dot(e1)
                - e0.dot(e0) * e1.dot(e2)
        );

        const double Y = - e0.cross(e1).dot(e2) * e0.dot(e0);

        const double C = atan2(Y, X);
        energy += 0.5 * data->_internal_edge_k_bend(i) * C * C;
    }
    return energy;
}

inline Eigen::Matrix<double, 9, 5>
CalculatePFPX(const Eigen::Vector3d &e0, const Eigen::Vector3d &e1, const Eigen::Vector3d &e2) {
    Eigen::Matrix<double, 9, 5> pFpX = Eigen::Matrix<double, 9, 5>::Zero();
    pFpX.col(0).segment<3>(0) = e1.cross(e2);
    pFpX.col(0).segment<3>(3) = e2.cross(e0);
    pFpX.col(0).segment<3>(6) = e0.cross(e1);
    pFpX.col(1).segment<3>(0) = 2 * e0;
    pFpX.col(2).segment<3>(0) = e1;
    pFpX.col(2).segment<3>(3) = e0;
    pFpX.col(3).segment<3>(0) = e2;
    pFpX.col(3).segment<3>(6) = e0;
    pFpX.col(4).segment<3>(3) = e2;
    pFpX.col(4).segment<3>(6) = e1;
    return pFpX;
}

template<class Data>
VectorXd ClothPhysics::GetPotentialGradient(const Data* data, const Ref<const VectorXd>& x) const {
    VectorXd gradient(x.size());
    gradient.setZero();
    for (int i = 0; i < data->_num_triangles; i++) {
        RowVector3i index = data->_face_topo.row(i);
        Vector3d xi = x.segment<3>(3 * index(0));
        Matrix<double, 3, 2> F;
        F.col(0) = x.segment<3>(3 * index(1)) - xi;
        F.col(1) = x.segment<3>(3 * index(2)) - xi;
        F *= data->_inv(i);
        Vector3d F1 = F.col(0);
        Vector3d F2 = F.col(1);

        Vector6d pCpF[3];
        double C[3];
        const double area = data->_area(i);
        C[0] = area * (F1.norm() - data->_stretch_u);
        pCpF[0].segment<3>(0) = area * F1.normalized();
        pCpF[0].segment<3>(3).setZero();
        C[1] = area * (F2.norm() - data->_stretch_v);
        pCpF[1].segment<3>(0).setZero();
        pCpF[1].segment<3>(3) = area * F2.normalized();
        C[2] = area * F1.dot(F2);
        pCpF[2].segment<3>(0) = area * F2;
        pCpF[2].segment<3>(3) = area * F1;
        Vector6d pCpX[3];
        const Matrix6d pFpX = data->_pFpx[i];
        for (int j = 0; j < 3; j++) {
            pCpX[j] = pFpX * pCpF[j];
        }
        Vector6d pEpX = data->_k_stretch * (C[0] * pCpX[0] + C[1] * pCpX[1]) + data->_k_shear * C[2] * pCpX[2];

        gradient.segment<3>(3 * index[1]) += pEpX.segment<3>(0);
        gradient.segment<3>(3 * index[2]) += pEpX.segment<3>(3);
        gradient.segment<3>(3 * index[0]) += - pEpX.segment<3>(0) - pEpX.segment<3>(3);
    }

    for (int i = 0; i < data->_num_internal_edges; i++) {
        RowVector4i index = data->_internal_edge.row(i);
        Vector3d xi = x.segment<3>(3 * index(0));
        Vector3d e0 = x.segment<3>(3 * index(1)) - xi;
        Vector3d e1 = x.segment<3>(3 * index(2)) - xi;
        Vector3d e2 = x.segment<3>(3 * index(3)) - xi;

        const double a = e0.cross(e1).dot(e2);
        const double b = e0.dot(e0);
        const double c = e0.dot(e1);
        const double d = e0.dot(e2);
        const double e = e1.dot(e2);
        const double N = data->_internal_edge_length(i);

        const double X = N * (c * d - b * e);
        const double Y = - a * b;

        const double C = atan2(Y, X);

        Vector5d pCpF = 1.0 / (X * X + Y * Y) * (Vector5d() <<
                - X * b,
                - X * a + N * Y * e,
                - N * Y * d,
                - N * Y * c,
                N * Y * b
        ).finished();

        Eigen::Matrix<double, 9, 5> pFpx = CalculatePFPX(e0, e1, e2);

        Vector9d pCpX = pFpx * pCpF;
        Vector9d pEpX = data->_internal_edge_k_bend(i) * C * pCpX;

        gradient.segment<3>(3 * index[1]) += pEpX.segment<3>(0);
        gradient.segment<3>(3 * index[2]) += pEpX.segment<3>(3);
        gradient.segment<3>(3 * index[3]) += pEpX.segment<3>(6);
        gradient.segment<3>(3 * index[0]) += - pEpX.segment<3>(0) - pEpX.segment<3>(3) - pEpX.segment<3>(6);
    }
    return gradient;
}

#include "Timer.h"

template<class Data>
void ClothPhysics::GetPotentialHessian(const Data* data, const Ref<const VectorXd>& x, COO &coo, int x_offset, int y_offset) const {
//    static double triangle_hessian_t = 0;
//    static int cnt = 0;
//    auto start_t = clock();
    for (int i = 0; i < data->_num_triangles; i++) {
        RowVector3i index = data->_face_topo.row(i);
        Vector3d xi = x.segment<3>(3 * index(0));
        Matrix<double, 3, 2> F;
        F.col(0) = x.segment<3>(3 * index(1)) - xi;
        F.col(1) = x.segment<3>(3 * index(2)) - xi;
        F *= data->_inv(i);
        const Vector3d F1 = F.col(0);
        const Vector3d F2 = F.col(1);
        const double F1_norm = F1.norm();
        const double F2_norm = F2.norm();

        const double area = data->_area(i);
        double C[3];
        C[0] = area * (F1_norm - data->_stretch_u);
        C[1] = area * (F2_norm - data->_stretch_v);
        C[2] = area * F1.dot(F2);

        Matrix3d F1F1T = F1 * F1.transpose();
        Matrix3d F1F2T = F1 * F2.transpose();
        Matrix3d F2F2T = F2 * F2.transpose();

        const double F1_norm3 = F1_norm * F1_norm * F1_norm;
        const double F2_norm3 = F2_norm * F2_norm * F2_norm;

        Matrix6d p2EpX2;

#ifdef BUILD_TEST
        p2EpX2.block<3, 3>(0, 0) = _k_stretch * area * area * (1 - _stretch_u / F1_norm) * Matrix3d::Identity()
                                 + _k_shear * area * area * F2F2T
                                 + _k_stretch * area * area * _stretch_u / F1_norm3 * F1F1T;
        p2EpX2.block<3, 3>(0, 3) = _k_shear * area * area * F1F2T.transpose() + _k_shear * C[2] * area * Matrix3d::Identity();
        p2EpX2.block<3, 3>(3, 0) = _k_shear * area * area * F1F2T + _k_shear * C[2] * area * Matrix3d::Identity();
        p2EpX2.block<3, 3>(3, 3) = _k_stretch * area * area * (1 - _stretch_v / F2_norm) * Matrix3d::Identity()
                                 + _k_shear * area * area * F1F1T
                                 + _k_stretch * area * area * _stretch_v / F2_norm3 * F2F2T;
#else
        p2EpX2.block<3, 3>(0, 0) = (data->_k_stretch * area * area * (1 - data->_stretch_u / F1_norm) + 0.5 * data->_k_shear * C[2] * area) * Matrix3d::Identity()
                                   + data->_k_shear * area * area * F2F2T
                                   + data->_k_stretch * area * area * data->_stretch_u / F1_norm3 * F1F1T;
        p2EpX2.block<3, 3>(0, 3) = data->_k_shear * area * area * F1F2T.transpose() + 0.5 * data->_k_shear * C[2] * area * Matrix3d::Identity();
        p2EpX2.block<3, 3>(3, 0) = data->_k_shear * area * area * F1F2T + 0.5 * data->_k_shear * C[2] * area * Matrix3d::Identity();
        p2EpX2.block<3, 3>(3, 3) = (data->_k_stretch * area * area * (1 - data->_stretch_v / F2_norm) + 0.5 * data->_k_shear * C[2] * area) * Matrix3d::Identity()
                                   + data->_k_shear * area * area * F1F1T
                                   + data->_k_stretch * area * area * data->_stretch_v / F2_norm3 * F2F2T;
#endif
        p2EpX2 = data->_pFpx(i) * p2EpX2 * data->_pFpx(i).transpose();

        Matrix9d p2EpX2_full = Matrix9d::Zero();
        p2EpX2_full.block<6, 6>(3, 3) = p2EpX2;
        p2EpX2_full.block<6, 3>(3, 0) = - p2EpX2.block<6, 3>(0, 0) - p2EpX2.block<6, 3>(0, 3);
        p2EpX2_full.block<3, 9>(0, 0) = - p2EpX2_full.block<3, 9>(3, 0) - p2EpX2_full.block<3, 9>(6, 0);

        for (int j = 0; j < 3; j++) {
            const int global_offset_j = index(j) * 3;
            const int local_offset_j = 3 * j;
            for (int k = 0; k < 3; k++) {
                const int global_offset_k = index(k) * 3;
                const int local_offset_k = 3 * k;
                for (int row = 0; row < 3; row++) {
                    for (int col = 0; col < 3; col++) {
                        coo.push_back(
                            Tripletd(
                                global_offset_j + row, global_offset_k + col,
                                p2EpX2_full(local_offset_j + row, local_offset_k + col)
                            )
                        );
                    }
                }
            }
        }
    }
//    triangle_hessian_t += 1.0 * (clock() - start_t) / CLOCKS_PER_SEC;
//    if (++cnt % 10000 == 0) {
//        spdlog::info("avg time for triangle hessian calculation: {}", triangle_hessian_t / 10000);
//        triangle_hessian_t = 0;
//    }

//    STOP_TIMING("Triangle Hessian", triangle_hessian)

//    START_TIMING(edge_hessian)
    for (int i = 0; i < data->_num_internal_edges; i++) {
        RowVector4i index = data->_internal_edge.row(i);
        const Vector3d xi = x.segment<3>(3 * index(0));
        const Vector3d e0 = x.segment<3>(3 * index(1)) - xi;
        const Vector3d e1 = x.segment<3>(3 * index(2)) - xi;
        const Vector3d e2 = x.segment<3>(3 * index(3)) - xi;

        const double a = e0.cross(e1).dot(e2);
        const double b = e0.dot(e0);
        const double c = e0.dot(e1);
        const double d = e0.dot(e2);
        const double e = e1.dot(e2);
        const double N = data->_internal_edge_length(i);

        const double X = N * (c * d - b * e);
        const double Y = - a * b;

        const double C = atan2(Y, X);

        Vector5d pCpF = 1.0 / (X * X + Y * Y) * (Vector5d() <<
                - X * b,
                - X * a + N * Y * e,
                - N * Y * d,
                - N * Y * c,
                N * Y * b
        ).finished();

        Eigen::Matrix<double, 9, 5> pFpx = CalculatePFPX(e0, e1, e2);
        Vector9d pCpx = pFpx * pCpF;

        const double square_diff = X * X - Y * Y;
        const double square_sum = X * X + Y * Y;

        Vector5d F1, F2;
        F1 << b, a, 0, 0, 0;
        F2 << 0, e, -d, -c, b;

        Vector5d p2CpFpX = (
                square_diff / (square_sum * square_sum) * F1
                - N * 2 * X * Y / (square_sum * square_sum) * F2
        );
        Vector5d p2CpFpY = (
                2 * X * Y / (square_sum * square_sum) * F1
                + N * square_diff / (square_sum * square_sum) * F2
        );

        Vector5d pXpF = -N * F2;
        Vector5d pYpF = -F1;

        Matrix5d p2CpF2 = Matrix5d::Zero();
        p2CpF2(0, 1) = p2CpF2(1, 0) = - X / square_sum;
        p2CpF2(1, 4) = p2CpF2(4, 1) = N * Y / square_sum;
        p2CpF2(2, 3) = p2CpF2(3, 2) = - N * Y / square_sum;

        p2CpF2 += p2CpFpX * pXpF.transpose() + p2CpFpY * pYpF.transpose();

        Matrix9d p2Cpx2 = Matrix9d::Zero();

        p2Cpx2.block<3, 3>(0, 3) = -HatMatrix(e2);
        p2Cpx2.block<3, 3>(0, 6) = HatMatrix(e1);
        p2Cpx2.block<3, 3>(3, 0) = HatMatrix(e2);
        p2Cpx2.block<3, 3>(3, 6) = -HatMatrix(e0);
        p2Cpx2.block<3, 3>(6, 0) = -HatMatrix(e1);
        p2Cpx2.block<3, 3>(6, 3) = HatMatrix(e0);
        p2Cpx2 *= pCpF(0);

        p2Cpx2.block<3, 3>(0, 0) += 2 * pCpF(1) * Matrix3d::Identity();
        p2Cpx2.block<3, 3>(0, 3) += pCpF(2) * Matrix3d::Identity();
        p2Cpx2.block<3, 3>(3, 0) += pCpF(2) * Matrix3d::Identity();
        p2Cpx2.block<3, 3>(0, 6) += pCpF(3) * Matrix3d::Identity();
        p2Cpx2.block<3, 3>(6, 0) += pCpF(3) * Matrix3d::Identity();
        p2Cpx2.block<3, 3>(3, 6) += pCpF(4) * Matrix3d::Identity();
        p2Cpx2.block<3, 3>(6, 3) += pCpF(4) * Matrix3d::Identity();

        p2Cpx2 += pFpx * p2CpF2 * pFpx.transpose();

        Matrix9d p2Epx2 = data->_internal_edge_k_bend(i) * pCpx * pCpx.transpose() + data->_internal_edge_k_bend(i) * C * p2Cpx2;
#ifndef BUILD_TEST
        p2Epx2 = PositiveProject(p2Epx2);
#endif
        Matrix12d p2Epx2_full;
        p2Epx2_full.block<9, 9>(3, 3) = p2Epx2;
        p2Epx2_full.block<9, 3>(3, 0) = - p2Epx2.block<9, 3>(0, 0) - p2Epx2.block<9, 3>(0, 3) - p2Epx2.block<9, 3>(0, 6);
        p2Epx2_full.block<3, 12>(0, 0) = - p2Epx2_full.block<3, 12>(3, 0) - p2Epx2_full.block<3, 12>(6, 0) - p2Epx2_full.block<3, 12>(9, 0);

        for (int j = 0; j < 4; j++) {
            const int global_offset_j = index(j) * 3;
            const int local_offset_j = j * 3;
            for (int k = 0; k < 4; k++) {
                const int global_offset_k = index(k) * 3;
                const int local_offset_k = k * 3;
                for (int row = 0; row < 3; row++) {
                    for (int col = 0; col < 3; col++) {
                        coo.push_back(Tripletd(
                            global_offset_j + row,
                            global_offset_k + col,
                            p2Epx2_full(local_offset_j + row, local_offset_k + col)
                        ));
                    }
                }
            }
        }
    }
//    STOP_TIMING("Edge Hessian", edge_hessian)
}