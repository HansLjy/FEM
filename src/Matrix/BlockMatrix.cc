#include "BlockMatrix.h"

void BlockMatrix::ToSparse(COO& coo, int x_offset, int y_offset) {
    int row_sub_offset = 0; // offset of current row in _submatrix
    for (int i = 0; i < _num_row_blocks; i++) {
        const int row_offset = _row_offsets[i];
        const int row_length = _row_segment_lengths[i];
        int col_sub_offset = 0;
        for (int j = 0; j < _num_column_blocks; j++) {
            const int column_offset = _column_offsets[j];
            const int column_length = _column_segment_lengths[j];
            DenseToCOO(
                _submatrix.block(row_sub_offset, col_sub_offset, row_length, column_length),
                coo, x_offset + row_offset, y_offset + column_offset
            );
            col_sub_offset += column_length;
        }
        row_sub_offset += row_length;
    }
}

BlockVector BlockVector::RightProduct(const Ref<MatrixXd>& rhs) const {
    return BlockVector(_rows, _num_row_blocks, _row_offsets, _row_segment_lengths, _submatrix * rhs);
}


void BlockVector::RightProduct(const Ref<const VectorXd>& rhs, Ref<VectorXd> result) const {
    int row_sub_offset = 0;
    for (int i = 0; i < _num_row_blocks; i++) {
        const int row_offset = _row_offsets[i];
        const int row_segment_length = _row_segment_lengths[i];
        result.segment(row_offset, row_segment_length) += _submatrix.middleRows(row_sub_offset, row_segment_length) * rhs;
        row_sub_offset += _row_segment_lengths[i];
    }
}

BlockMatrix BlockVector::RightTransposeProduct(const BlockVector& rhs) {
    return {
        _rows, rhs._rows, _num_row_blocks, rhs._num_row_blocks,
        _row_offsets, _row_segment_lengths,
        rhs._row_offsets, rhs._row_segment_lengths,
        _submatrix * rhs._submatrix.transpose()
    };
}