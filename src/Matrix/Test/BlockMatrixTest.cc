#include "gtest/gtest.h"
#include "BlockMatrix.h"

class FriendlyBlockVector : public BlockVector {
public:
    using BlockVector::BlockVector;
    FRIEND_TEST(BlockMatrixTest, BlockVectorTest);
};

MatrixXd GenerateDenseMatrix(int rows, int cols, int num_row_blocks,
                             const std::vector<int>& row_offsets,
                             const std::vector<int>& row_segment_lengths,
                             const Ref<const MatrixXd>& submatrix) {
    MatrixXd dense_matrix = MatrixXd::Zero(rows, cols);
    int row_sub_offset = 0;
    for (int i = 0; i < num_row_blocks; i++) {
        dense_matrix.middleRows(row_offsets[i], row_segment_lengths[i]) += submatrix.middleRows(row_sub_offset, row_segment_lengths[i]);
        row_sub_offset += row_segment_lengths[i];
    }
    return dense_matrix;
}

TEST(BlockMatrixTest, BlockVectorTest) {
    MatrixXd submatrix = MatrixXd::Random(5, 3);
    FriendlyBlockVector vector(10, 2, {0, 5}, {2, 3}, submatrix);
    MatrixXd dense_vector = GenerateDenseMatrix(10, 3, 2, {0, 5}, {2, 3}, submatrix);

    Vector3d rhs_vec = Vector3d::Random();
    VectorXd result_vec = VectorXd::Zero(10);
    vector.RightProduct(rhs_vec, result_vec);
    VectorXd result_vec_correct = dense_vector * rhs_vec;

    EXPECT_TRUE(result_vec.isApprox(result_vec_correct));
    
    Matrix3d rhs_mat = Matrix3d::Random();
    vector.RightProductInPlace(rhs_mat);
    dense_vector *= rhs_mat;
    MatrixXd result_mat = GenerateDenseMatrix(
        vector._rows, 3, vector._num_row_blocks,
        vector._row_offsets, vector._row_segment_lengths,
        vector._submatrix
    );

    EXPECT_TRUE(dense_vector.isApprox(result_mat));
}