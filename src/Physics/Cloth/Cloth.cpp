//
// Created by hansljy on 10/27/22.
//


#include "Cloth.h"
#include "ClothShape/ClothShape.h"
#include "unsupported/Eigen/KroneckerProduct"
#include "JsonUtil.h"

DEFINE_CLONE(Object, Cloth)

Cloth::Cloth(const json &config)
    : Cloth(config["density"], config["k-stretch"], config["k-shear"], config["k-bend"],
            Json2Vec(config["start"]), Json2Vec(config["u-end"]), Json2Vec(config["v-end"]),
            config["u-segments"], config["v-segments"], config["stretch-u"], config["stretch-v"]){}

Cloth::Cloth(double rho, double k_stretch, double k_shear, double k_bend, const Eigen::Vector3d &start,
             const Eigen::Vector3d &u_end, const Eigen::Vector3d &v_end, int num_u_segments, int num_v_segments,
             double stretch_u, double stretch_v)
             : Cloth(rho, k_stretch, k_shear, k_bend,
                     GeneratePosition(start, u_end, v_end, num_u_segments, num_v_segments),
                     GenerateUVCoord(start, u_end, v_end, num_u_segments, num_v_segments),
                     GenerateTopo(num_u_segments, num_v_segments),
                     stretch_u, stretch_v) {}

Cloth::Cloth(double rho, double k_stretch, double k_shear, double k_bend, const VectorXd &x, const VectorXd &uv_corrd,
             const MatrixXi &topo, double stretch_u, double stretch_v)
             : Object(x), ShapedObject(ClothShape()), SampledObject(GenerateMass(rho, uv_corrd, topo)),
               _num_points(x.size() / 3),
               _num_triangles(topo.rows()),
               _k_stretch(k_stretch), _k_shear(k_shear), _k_bend(k_bend),
               _stretch_u(stretch_u), _stretch_v(stretch_v),
               _uv_coord(uv_corrd), _topo(topo)
               {
    _area.resize(_num_triangles);
    for (int i = 0; i < _num_triangles; i++) {
        RowVector3i index = _topo.row(i);
        Vector2d e1 = uv_corrd.segment<2>(index(1) * 2) - uv_corrd.segment<2>(index(0) * 2);
        Vector2d e2 = uv_corrd.segment<2>(index(2) * 2) - uv_corrd.segment<2>(index(0) * 2);
        const double S = (e1(0) * e2(1) - e1(1) * e2(0)) / 2;
        if (S > 0) {
            _area(i) = S;
        } else {
            _area(i) = -S;
            std::swap(_topo(i, 1), _topo(i, 2));
        }
    }

    std::vector<std::tuple<int, int, int>> edges;

    _inv.resize(_num_triangles);
    _pFpx.resize(_num_triangles);
    for (int i = 0; i < _num_triangles; i++) {
        RowVector3i index = _topo.row(i);
        Vector2d e1 = uv_corrd.segment<2>(index(1) * 2) - uv_corrd.segment<2>(index(0) * 2);
        Vector2d e2 = uv_corrd.segment<2>(index(2) * 2) - uv_corrd.segment<2>(index(0) * 2);

        Matrix2d F;
        F.col(0) = e1;
        F.col(1) = e2;

        _inv(i) = F.inverse();
        _pFpx(i) = Eigen::KroneckerProduct(_inv(i), Matrix3d::Identity());

        for (int j = 0; j < 3; j++) {
            edges.push_back(
                std::make_tuple(
                    std::min(index(j), index((j + 1) % 3)),
                    std::max(index(j), index((j + 1) % 3)),
                    index((j + 2) % 3)
                )
            );
        }
    }

    std::sort(edges.begin(), edges.end());
    const int num_edges = edges.size();
    int num_internal_edges = 0;
    for (int i = 0; i < num_edges - 1; i++) {
        if (std::get<0>(edges[i]) == std::get<0>(edges[i + 1])
            && std::get<1>(edges[i]) == std::get<1>(edges[i + 1])) {
            // internal edge
            num_internal_edges++;
        }
    }
    _internal_edge.resize(num_internal_edges, 4);
    _internal_edge_length.resize(num_internal_edges);
    _num_internal_edges = 0;
    for (int i = 0; i < num_edges - 1; i++) {
        if (std::get<0>(edges[i]) == std::get<0>(edges[i + 1])
            && std::get<1>(edges[i]) == std::get<1>(edges[i + 1])) {
            _internal_edge.row(_num_internal_edges)
                << std::get<0>(edges[i]), std::get<1>(edges[i]),
                   std::get<2>(edges[i]), std::get<2>(edges[i + 1]);
            _internal_edge_length(_num_internal_edges) = (
                    _x.segment<3>(3 * std::get<1>(edges[i])) -
                    _x.segment<3>(3 * std::get<0>(edges[i]))
            ).norm();
            _num_internal_edges++;
        }
    }

}

double Cloth::GetPotential(const Ref<const Eigen::VectorXd> &x) const {
    double energy = 0;
    for (int i = 0; i < _num_triangles; i++) {
        RowVector3i index = _topo.row(i);
        Matrix<double, 3, 2> F;
        F.col(0) = x.segment<3>(3 * index(1)) - x.segment<3>(3 * index(0));
        F.col(1) = x.segment<3>(3 * index(2)) - x.segment<3>(3 * index(0));
        F = F * _inv(i);
        Vector3d F1 = F.col(0);
        Vector3d F2 = F.col(1);
        const double C1 = (F1.norm() - _stretch_u) * _area(i);
        const double C2 = (F2.norm() - _stretch_v) * _area(i);
        const double C3 = _area(i) * F1.dot(F2);

        energy += 0.5 * _k_stretch * (C1 * C1 + C2 * C2) + 0.5 * _k_shear * C3 * C3;
    }

    for (int i = 0; i < _num_internal_edges; i++) {
        RowVector4i edge_index = _internal_edge.row(i);
        Vector3d xi = x.segment<3>(3 * edge_index(0));
        Vector3d e0 = x.segment<3>(3 * edge_index(1)) - xi;
        Vector3d e1 = x.segment<3>(3 * edge_index(2)) - xi;
        Vector3d e2 = x.segment<3>(3 * edge_index(3)) - xi;

        const double X = _internal_edge_length(i) * (
                e0.dot(e2) * e0.dot(e1)
                - e0.dot(e0) * e1.dot(e2)

        );
        const double Y = - e0.cross(e1).dot(e2) * e0.dot(e0);

        const double C = atan2(Y, X);
        energy += 0.5 * _k_bend * C * C;
    }
    return energy;
}

Eigen::Matrix<double, 9, 5>
Cloth::CalculatePFPX(const Eigen::Vector3d &e0, const Eigen::Vector3d &e1, const Eigen::Vector3d &e2) {
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

VectorXd Cloth::GetPotentialGradient() const {
    VectorXd gradient(_x.size());
    gradient.setZero();
    for (int i = 0; i < _num_triangles; i++) {
        RowVector3i index = _topo.row(i);
        Vector3d xi = _x.segment<3>(3 * index(0));
        Matrix<double, 3, 2> F;
        F.col(0) = _x.segment<3>(3 * index(1)) - xi;
        F.col(1) = _x.segment<3>(3 * index(2)) - xi;
        F *= _inv(i);
        Vector3d F1 = F.col(0);
        Vector3d F2 = F.col(1);

        Vector6d pCpF[3];
        double C[3];
        const double area = _area(i);
        C[0] = area * (F1.norm() - _stretch_u);
        pCpF[0].segment<3>(0) = area * F1.normalized();
        pCpF[0].segment<3>(3).setZero();
        C[1] = area * (F2.norm() - _stretch_v);
        pCpF[1].segment<3>(0).setZero();
        pCpF[1].segment<3>(3) = area * F2.normalized();
        C[2] = area * F1.dot(F2);
        pCpF[2].segment<3>(0) = area * F2;
        pCpF[2].segment<3>(3) = area * F1;
        Vector6d pCpX[3];
        const Matrix6d pFpX = _pFpx[i];
        for (int j = 0; j < 3; j++) {
            pCpX[j] = pFpX * pCpF[j];
        }
        Vector6d pEpX = _k_stretch * (C[0] * pCpX[0] + C[1] * pCpX[1]) + _k_shear * C[2] * pCpX[2];

        gradient.segment<3>(3 * index[1]) += pEpX.segment<3>(0);
        gradient.segment<3>(3 * index[2]) += pEpX.segment<3>(3);
        gradient.segment<3>(3 * index[0]) += - pEpX.segment<3>(0) - pEpX.segment<3>(3);
    }

    for (int i = 0; i < _num_internal_edges; i++) {
        RowVector4i index = _internal_edge.row(i);
        Vector3d xi = _x.segment<3>(3 * index(0));
        Vector3d e0 = _x.segment<3>(3 * index(1)) - xi;
        Vector3d e1 = _x.segment<3>(3 * index(2)) - xi;
        Vector3d e2 = _x.segment<3>(3 * index(3)) - xi;

        const double a = e0.cross(e1).dot(e2);
        const double b = e0.dot(e0);
        const double c = e0.dot(e1);
        const double d = e0.dot(e2);
        const double e = e1.dot(e2);
        const double N = _internal_edge_length(i);

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
        Vector9d pEpX = _k_bend * C * pCpX;

        gradient.segment<3>(3 * index[1]) += pEpX.segment<3>(0);
        gradient.segment<3>(3 * index[2]) += pEpX.segment<3>(3);
        gradient.segment<3>(3 * index[3]) += pEpX.segment<3>(6);
        gradient.segment<3>(3 * index[0]) += - pEpX.segment<3>(0) - pEpX.segment<3>(3) - pEpX.segment<3>(6);
    }
    return gradient;
}

#include "Timer.h"

void Cloth::GetPotentialHessian(COO &coo, int x_offset, int y_offset) const {
    START_TIMING(triangle_hessian)
    for (int i = 0; i < _num_triangles; i++) {
        RowVector3i index = _topo.row(i);
        Vector3d xi = _x.segment<3>(3 * index(0));
        Matrix<double, 3, 2> F;
        F.col(0) = _x.segment<3>(3 * index(1)) - xi;
        F.col(1) = _x.segment<3>(3 * index(2)) - xi;
        F *= _inv(i);
        const Vector3d F1 = F.col(0);
        const Vector3d F2 = F.col(1);
        const double F1_norm = F1.norm();
        const double F2_norm = F2.norm();

        const double area = _area(i);
        double C[3];
        C[0] = area * (F1_norm - _stretch_u);
        C[1] = area * (F2_norm - _stretch_v);
        C[2] = area * F1.dot(F2);

        Vector6d pCpF[3];

        pCpF[0].segment<3>(0) = area * F1.normalized();
        pCpF[0].segment<3>(3).setZero();
        pCpF[1].segment<3>(0).setZero();
        pCpF[1].segment<3>(3) = area * F2.normalized();
        pCpF[2].segment<3>(0) = area * F2;
        pCpF[2].segment<3>(3) = area * F1;

        Matrix6d p2CpF2_total1;

        p2CpF2_total1.block<3, 3>(0, 0) = _k_stretch * C[0] * area * (Matrix3d::Identity() / F1_norm - F1 * F1.transpose() / (F1_norm * F1_norm * F1_norm));
        p2CpF2_total1.block<3, 3>(3, 3) = _k_stretch * C[1] * area * (Matrix3d::Identity() / F2_norm - F2 * F2.transpose() / (F2_norm * F2_norm * F2_norm));
        p2CpF2_total1.block<3, 3>(0, 3) = p2CpF2_total1.block<3, 3>(3, 0) = _k_shear * C[2] *  area * Matrix3d::Identity();

        Matrix6d p2CpF2_total =
                _k_stretch * (pCpF[0] * pCpF[0].transpose() + pCpF[1] * pCpF[1].transpose())
              + _k_shear * pCpF[2] * pCpF[2].transpose()
              + p2CpF2_total1;

        Matrix6d pFpX = _pFpx(i);
        Matrix6d p2EpX2 = pFpX * p2CpF2_total * pFpX.transpose();
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
    STOP_TIMING("Triangle Hessian", triangle_hessian)

    START_TIMING(edge_hessian)
    for (int i = 0; i < _num_internal_edges; i++) {
        RowVector4i index = _internal_edge.row(i);
        const Vector3d xi = _x.segment<3>(3 * index(0));
        const Vector3d e0 = _x.segment<3>(3 * index(1)) - xi;
        const Vector3d e1 = _x.segment<3>(3 * index(2)) - xi;
        const Vector3d e2 = _x.segment<3>(3 * index(3)) - xi;

        const double a = e0.cross(e1).dot(e2);
        const double b = e0.dot(e0);
        const double c = e0.dot(e1);
        const double d = e0.dot(e2);
        const double e = e1.dot(e2);
        const double N = _internal_edge_length(i);

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

        Matrix9d p2Epx2 = _k_bend * pCpx * pCpx.transpose() + _k_bend * C * p2Cpx2;
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
    STOP_TIMING("Edge Hessian", edge_hessian)
}

VectorXd
Cloth::GeneratePosition(const Eigen::Vector3d &start, const Eigen::Vector3d &u_end, const Eigen::Vector3d &v_end,
                        int num_u_segments, int num_v_segments) {
    Vector3d delta_u = (u_end - start) / num_u_segments;
    Vector3d delta_v = (v_end - start) / num_v_segments;
    VectorXd position(3 * (num_u_segments + 1) * (num_v_segments + 1));

    /**
     *  v-end
     *     |
     *    U+1
     *     |
     *     0 -- 1 -- 2 -- ...
     *  start                u-end
     */
    for (int i = 0; i <= num_v_segments; i++) {
        for (int j = 0; j <= num_u_segments; j++) {
            position.segment<3>(3 * (i * (num_u_segments + 1) + j)) = start + j * delta_u + i * delta_v;
        }
    }
    return position;
}

VectorXd
Cloth::GenerateUVCoord(const Eigen::Vector3d &start, const Eigen::Vector3d &u_end, const Eigen::Vector3d &v_end,
                       int num_u_segments, int num_v_segments) {
    Vector3d norm = (u_end - start).cross(v_end - start);
    Vector3d u_direction = (u_end - start).normalized();
    Vector3d v_direction = norm.cross(u_end - start).normalized();
    const Vector2d delta_u = (Vector2d() << (u_end - start).dot(u_direction), (u_end - start).dot(v_direction)).finished() / num_u_segments;
    const Vector2d delta_v = (Vector2d() << (v_end - start).dot(u_direction), (v_end - start).dot(v_direction)).finished() / num_v_segments;

    VectorXd uv_coord(2 * (num_u_segments + 1) * (num_v_segments + 1));
    for (int i = 0; i <= num_v_segments; i++) {
        for (int j = 0; j <= num_u_segments; j++) {
            uv_coord.segment<2>(2 * (i * (num_u_segments + 1) + j)) = i * delta_v + j * delta_u;
        }
    }
    return uv_coord;
}

MatrixXi Cloth::GenerateTopo(int num_u_segments, int num_v_segments) {
    MatrixXi topo(num_u_segments * num_v_segments * 2, 3);
    for (int i = 0; i < num_v_segments; i++) {
        for (int j = 0; j < num_u_segments; j++) {
            // row i, column j
            const int lower_left_id = i * (num_u_segments + 1) + j;
            topo.row(2 * (i * num_u_segments + j))
                << lower_left_id, lower_left_id + 1, lower_left_id + num_u_segments + 1;
            topo.row(2 * (i * num_u_segments + j) + 1)
                << lower_left_id + 1, lower_left_id + num_u_segments + 1, lower_left_id + num_u_segments + 2;
        }
    }
    return topo;
}

VectorXd Cloth::GenerateMass(double rho, const VectorXd &uv_coord, const MatrixXi &topo) {
    int num_triangles = topo.rows();
    int num_points = uv_coord.size() / 2;
    VectorXd mass(num_points);
    mass.setZero();
    for (int i = 0; i < num_triangles; i++) {
        RowVector3i index = topo.row(i);
        Vector2d e1 = uv_coord.segment<2>(index(1) * 2) - uv_coord.segment<2>(index(0) * 2);
        Vector2d e2 = uv_coord.segment<2>(index(2) * 2) - uv_coord.segment<2>(index(0) * 2);
        const double S = abs(e1(0) * e2(1) - e1(1) * e2(0)) / 2;
        for (int j = 0; j < 3; j++) {
            mass(index(j)) += S * rho / 3;
        }
    }
    return mass;
}