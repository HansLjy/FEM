#pragma once
#include "EigenAll.h"
#include <array>

class BlockMatrix {
public:
    BlockMatrix(int rows, int cols,
                int num_row_blocks, int num_column_blocks,
                const std::vector<int>& row_offsets,
                const std::vector<int>& row_segment_lengths,
                const std::vector<int>& column_offsets,
                const std::vector<int>& column_segment_lengths,
                const Ref<const MatrixXd>& submatrix)
                : _rows(rows), _cols(cols),
                  _num_row_blocks(num_row_blocks), _num_column_blocks(num_column_blocks),
                  _row_offsets(row_offsets), _row_segment_lengths(row_segment_lengths),
                  _column_offsets(column_offsets), _column_segment_lengths(column_segment_lengths),
                  _submatrix(submatrix) {}

    void ToSparse(COO& coo, int x_offset, int y_offset);

protected:
    int _rows, _cols;
    int _num_row_blocks, _num_column_blocks;
    std::vector<int> _row_offsets;
    std::vector<int> _row_segment_lengths;
    std::vector<int> _column_offsets;
	std::vector<int> _column_segment_lengths;

    MatrixXd _submatrix;
};

class BlockVector {
public:
    BlockVector(int rows, int num_row_blocks,
                const std::vector<int>& row_offsets,
                const std::vector<int>& row_segment_lengths,
                const Ref<const MatrixXd>& submatrix)
                : _rows(rows), _num_row_blocks(num_row_blocks), _row_offsets(row_offsets), _row_segment_lengths(row_segment_lengths), _submatrix(submatrix) {}

	BlockVector(const BlockVector& rhs) = default;
    
    BlockVector RightProduct(const Ref<MatrixXd>& rhs) const;
    void RightProduct(const Ref<const VectorXd>& rhs, Ref<VectorXd> result) const;

	//<- this * rhs.transpose()
    BlockMatrix RightTransposeProduct(const BlockVector& rhs);

    int _rows;
    int _num_row_blocks;
    std::vector<int> _row_offsets;
    std::vector<int> _row_segment_lengths;
    MatrixXd _submatrix;
};