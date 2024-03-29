//
// Created by hansljy on 10/11/22.
//

#include "EigenAll.h"

Matrix3d HatMatrix(const Vector3d& vec) {
    Matrix3d hat;
    hat << 0, -vec(2), vec(1),
            vec(2), 0, -vec(0),
            -vec(1), vec(0), 0;
    return hat;
}

// WARNING: this function does not check whether vec is zero or not
Vector3d FindPerpendicular(const Vector3d& vec) {
    Vector3d answer;
    if (vec(1) != 0 || vec(2) != 0) {
        answer << 0, -vec(2), vec(1);
    } else {
        answer << -vec(1), vec(0), 0;
    }
    return answer.normalized();
}

Matrix<double, 9, 3> GetVecHatMatrix() {
    Matrix<double, 9, 3> vec_hat_matrix;
    vec_hat_matrix <<
                   0, 0, 0,
            0, 0, 1,
            0, -1, 0,
            0, 0, -1,
            0, 0, 0,
            1, 0, 0,
            0, 1, 0,
            -1, 0, 0,
            0, 0, 0;
    return vec_hat_matrix;
}

Vector3d SkewVector(const Matrix3d& mat) {
    Matrix3d skew_matrix = (mat - mat.transpose()) / 2;
    return (Vector3d() << skew_matrix(2, 1), skew_matrix(0, 2), skew_matrix(1, 0)).finished();
}

void SparseToCOO(const SparseMatrixXd& mat, COO& coo, int offset_x, int offset_y) {
    for (int k = 0; k < mat.outerSize(); k++) {
        for (SparseMatrixXd::InnerIterator itr(mat, k); itr; ++itr) {
            coo.push_back(Tripletd(itr.row() + offset_x, itr.col() + offset_y, itr.value()));
        }
    }
}