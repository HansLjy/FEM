#include "gtest/gtest.h"
#include "BlockMatrix.h"

class FriendlyBlockVector : public BlockVector {
public:
    using BlockVector::BlockVector;
    FriendlyBlockVector(const BlockVector& rhs) : BlockVector(rhs) {}
    MatrixXd ToDenseMatrix(int cols) {
        MatrixXd dense_matrix = MatrixXd::Zero(_rows, cols);
        int row_sub_offset = 0;
        for (int i = 0; i < _num_row_blocks; i++) {
            dense_matrix.middleRows(_row_offsets[i], _row_segment_lengths[i]) += _submatrix.middleRows(row_sub_offset, _row_segment_lengths[i]);
            row_sub_offset += _row_segment_lengths[i];
        }
        return dense_matrix;
    }
    FRIEND_TEST(BlockMatrixTest, BlockVectorTest);
};

class FriendlyBlockMatrix : public BlockMatrix {
public:
    using BlockMatrix::BlockMatrix;
    FriendlyBlockMatrix(const BlockMatrix& rhs) : BlockMatrix(rhs) {}
};

const int cols = 3;

TEST(BlockMatrixTest, BlockVectorTest) {
    MatrixXd submatrix = MatrixXd::Random(5, cols);
    FriendlyBlockVector vector(10, 2, {0, 7}, {2, 3}, submatrix);
    MatrixXd dense_vector = vector.ToDenseMatrix(cols);

    Vector3d rhs_vec = Vector3d::Random();
    VectorXd result_vec = VectorXd::Random(10);
    VectorXd result_vec_origin = result_vec;
    vector.RightProduct(rhs_vec, result_vec);
    VectorXd result_vec_correct = dense_vector * rhs_vec;

    EXPECT_TRUE((result_vec - result_vec_origin).isApprox(result_vec_correct));
    
    Matrix3d rhs_mat = Matrix3d::Random();
    MatrixXd result_mat = FriendlyBlockVector(vector.RightProduct(rhs_mat)).ToDenseMatrix(cols);
    MatrixXd result_mat_correct = dense_vector * rhs_mat;

    EXPECT_TRUE(result_mat.isApprox(result_mat_correct));
}

TEST(BlockMatrixTest, BlockMatrixTest) {
    const int result_rows = 10;
    const int result_cols = 11;
    MatrixXd submatrixA = MatrixXd::Random(5, cols);
    MatrixXd submatrixB = MatrixXd::Random(6, cols);
    FriendlyBlockVector vectorA(result_rows, 2, {0, 5}, {2, 3}, submatrixA);
    MatrixXd dense_vector_A = vectorA.ToDenseMatrix(cols);
    FriendlyBlockVector vectorB(result_cols, 3, {1, 4, 9}, {1, 3, 2}, submatrixB);
    MatrixXd dense_vector_B = vectorB.ToDenseMatrix(cols);

    COO coo;
    const int result_blank_rows = 1;
    const int result_blank_cols = 2;
    vectorA.RightTransposeProduct(vectorB).ToSparse(coo, result_blank_rows, result_blank_cols);
    SparseMatrixXd result_sparse(result_rows + result_blank_rows, result_cols + result_blank_cols);
    result_sparse.setFromTriplets(coo.begin(), coo.end());
    MatrixXd result_dense = result_sparse.toDense();

    std::cerr << result_dense;

    MatrixXd result_correct = dense_vector_A * dense_vector_B.transpose();
    EXPECT_TRUE(result_correct.isApprox(result_dense.bottomRightCorner(result_rows, result_cols)));
}