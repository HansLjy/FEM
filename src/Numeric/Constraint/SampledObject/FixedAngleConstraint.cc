//
// Created by hansljy on 11/14/22.
//

#include "FixedAngleConstraint.h"

DEFINE_CLONE(Constraint, FixedAngleConstraint)

FixedAngleConstraint::FixedAngleConstraint(const Eigen::Vector3d &direction, int object_id, int point_id1,
                                           int point_id2)
                                           : Constraint(1, 1, {object_id}),
                                             _point_offset1(point_id1 * 3), _point_offset2(point_id2 * 3),
                                             _direction(direction.normalized()) {}

VectorXd FixedAngleConstraint::GetValue(const Eigen::VectorXd &x) const {
    int offset = _object_offsets[0];
    Vector3d cur_direction = x.segment<3>(offset + _point_offset2) - x.segment<3>(offset + _point_offset1);
    VectorXd value(1);
    value << _direction.dot(cur_direction.normalized()) - 1;
    return value;
}

void FixedAngleConstraint::GetGradient(const Eigen::VectorXd &x, COO &coo, int x_offset) const {
    const int object_offset = _object_offsets[0];
    const Vector3d cur_direction = x.segment<3>(object_offset + _point_offset2) - x.segment<3>(object_offset + _point_offset1);
    const double cur_direction_norm = cur_direction.norm();
    const Vector3d pfpx1 = - _direction / cur_direction_norm
            + _direction.dot(cur_direction) * cur_direction / (cur_direction_norm * cur_direction_norm * cur_direction_norm);
    for (int i = 0; i < 3; i++) {
        coo.push_back(Tripletd(x_offset, object_offset + _point_offset1 + i, pfpx1(i)));
        coo.push_back(Tripletd(x_offset, object_offset + _point_offset2 + i, -pfpx1(i)));
    }
}