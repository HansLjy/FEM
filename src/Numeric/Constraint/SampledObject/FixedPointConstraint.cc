//
// Created by hansljy on 10/19/22.
//

#include "FixedPointConstraint.h"

FixedPointConstraint::FixedPointConstraint(const Vector3d &fixed_point, int object_id, int point_id)
    : Constraint(1, 3, {object_id}), _point_offset(point_id * 3), _fixed_point(fixed_point) {}

VectorXd FixedPointConstraint::GetValue(const VectorXd &x) const {
    return x.segment<3>(_object_offsets[0] + _point_offset) - _fixed_point;
}

void FixedPointConstraint::GetGradient(const Eigen::VectorXd &x, COO &coo, int x_offset) const {
    for (int i = 0; i < 3; i++) {
        coo.push_back(Tripletd(x_offset + i, _object_offsets[0] + _point_offset + i, 1));
    }
}