#include "PDClothEnergyModel.hpp"
#include "GeometryUtil.hpp"
#include <iostream>

double PDEnergyModelFunction::GetPDSpringEnergy(
    const Ref<const VectorXd> &x,
    const Ref<const MatrixXi> &edge_topo,
    const std::vector<double> &rest_length, 
    double stiffness
) {
    const int num_edges = edge_topo.rows();
    double energy = 0;
    for (int i = 0; i < num_edges; i++) {
        const RowVector2i indices = edge_topo.row(i);
        double delta_length = (
            x.segment<3>(indices(0) * 3) -
            x.segment<3>(indices(1) * 3)
        ).norm() - rest_length[i];
        energy += 0.5 * stiffness * delta_length * delta_length;
    }
    return energy;
}

void PDEnergyModelFunction::GetPDSpringEnergyLHSMatrix(
    const Ref<const MatrixXi> &edge_topo,
    double stiffness,
    COO &coo, int x_offset, int y_offset) {
    const int num_edges = edge_topo.rows();
    for (int i = 0; i < num_edges; i++) {
        const RowVector2i indices = edge_topo.row(i);
        for (int j = 0; j < 3; j++) {
            coo.push_back(Tripletd(
                x_offset + indices[0] * 3 + j,
                y_offset + indices[0] * 3 + j,
                stiffness
            ));
            coo.push_back(Tripletd(
                x_offset + indices[0] * 3 + j,
                y_offset + indices[1] * 3 + j,
                -stiffness
            ));
            coo.push_back(Tripletd(
                x_offset + indices[1] * 3 + j,
                y_offset + indices[0] * 3 + j,
                -stiffness
            ));
            coo.push_back(Tripletd(
                x_offset + indices[1] * 3 + j,
                y_offset + indices[1] * 3 + j,
                stiffness
            ));
        }
    }
}

Vector3d PDEnergyModelFunction::SpringLocalProject(
    const Vector3d &x1, const Vector3d &x2, double rest_length
) {
    return (x1 - x2).normalized() * rest_length;
}

void PDEnergyModelFunction::GetPDSpringEnergyRHSVector(
    const Ref<const VectorXd> &x,
    const Ref<const MatrixXi> &edge_topo,
    const std::vector<double> &rest_length,
    double stiffness,
    Ref<VectorXd> y
) {
    const int num_edges = edge_topo.rows();
    for (int i = 0; i < num_edges; i++) {
        const RowVector2i indices = edge_topo.row(i);
        const Vector3d x1 = x.segment<3>(3 * indices[0]);
        const Vector3d x2 = x.segment<3>(3 * indices[1]);
        Vector3d d = SpringLocalProject(x1, x2, rest_length[i]);
        y.segment<3>(indices[0] * 3) += d * stiffness;
        y.segment<3>(indices[1] * 3) += -d * stiffness;
    }
}

void PDEnergyModelFunction::InitQuadraticBendingMatrix(
    const Ref<const VectorXd> &x,
    const Ref<const MatrixXi> &internal_edge_topo,
    double stiffness,
    SparseMatrixXd &Q
) {
    COO coo;

    const int num_internal_edges = internal_edge_topo.rows();
    for (int i = 0; i < num_internal_edges; i++) {
        const RowVector4i indices = internal_edge_topo.row(i);
        const Vector3d x0 = x.segment<3>(indices[0] * 3);
        const Vector3d x1 = x.segment<3>(indices[1] * 3);
        const Vector3d x2 = x.segment<3>(indices[2] * 3);
        const Vector3d x3 = x.segment<3>(indices[3] * 3);
        const double area1 = (x1 - x0).cross(x2 - x0).norm() / 2;
        const double area2 = (x1 - x0).cross(x3 - x0).norm() / 2;
        const double c01 = GeometryUtil::GetCot(x1 - x0, x2 - x0);
        const double c02 = GeometryUtil::GetCot(x1 - x0, x3 - x0);
        const double c03 = GeometryUtil::GetCot(x0 - x1, x2 - x1);
        const double c04 = GeometryUtil::GetCot(x0 - x1, x3 - x1);

        const double K[4] = {
            c03 + c04,
            c01 + c02,
            - c01 - c03,
            - c02 - c04
        };

        for (int block_row = 0; block_row < 4; block_row++) {
            const int row_offset = indices[block_row] * 3;
            for (int block_col = 0; block_col < 4; block_col++) {
                const int col_offset = indices[block_col] * 3;
                const double val = stiffness * K[block_row] * K[block_col] * 3 / (area1 + area2);
                for (int j = 0; j < 3; j++) {
                    coo.push_back(Tripletd(
                        row_offset + j,
                        col_offset + j,
                        val
                    ));
                }
            }
        }
    }

    Q.resize(x.size(), x.size());
    Q.setFromTriplets(coo.begin(), coo.end());
}

double PDEnergyModelFunction::GetPDQuadraticEnergy(
    const Ref<const VectorXd> &x,
    const SparseMatrixXd &Q
) {
    return x.transpose() * Q.selfadjointView<Eigen::Upper>() * x;
}

void PDEnergyModelFunction::GetPDQuadraticEnergyLHSMatrix(
    const SparseMatrixXd &Q,
    COO &coo, int x_offset, int y_offset
) {
    SparseToCOO(Q, coo, x_offset, y_offset);
}

void PDEnergyModelFunction::GetPDQuadraticEnergyRHSVector(Ref<VectorXd> y) {
    
}