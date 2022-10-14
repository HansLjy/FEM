//
// Created by hansljy on 10/11/22.
//

#include "FixedPoint.h"
#include "spdlog/spdlog.h"

DEFINE_CLONE(Constraint, FixedPoint)

FixedPoint::FixedPoint(const Curve &curve, int curve_idx, int fixed_point_idx, const Eigen::Vector3d &fixed_point)
    : Constraint(), _curve_idx(curve_idx),
      _fixed_point_idx(fixed_point_idx),
      _fixed_point(fixed_point) {}

int FixedPoint::GetSize() const {
    return 3;
}

int FixedPoint::GetObjectsNum() const {
    return 1;
}

int FixedPoint::GetObjectIndex(int object_id) const {
    return _curve_idx;
}

void FixedPoint::SetOffset(int offset_id, int offset) {
    if(offset_id != 0) {
        spdlog::warn("Invalid offset id! No offset was updated.");
    } else {
        _curve_offset = offset;
    }
}

VectorXd FixedPoint::GetValue(const Eigen::VectorXd &x) const {
    return x.block<3, 1>(_curve_offset + _fixed_point_idx * 3, 0) - _fixed_point;
}

void FixedPoint::GetGradient(const Eigen::VectorXd &x, COO &coo, int x_offset) const {
    for (int i = 0; i < 3; i++) {
        coo.push_back(Tripletd(x_offset + i, _curve_offset + 3 * _fixed_point_idx + i, 1));
    }
}