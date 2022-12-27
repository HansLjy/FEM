//
// Created by hansljy on 11/2/22.
//

#include "JointConstraint.h"

JointConstraint::JointConstraint(int object1_id, int point1_id, int object2_id, int point2_id)
    : Constraint(2, 3, {object1_id, object2_id}),
      _point_offsets({point1_id * 3, point2_id * 3}){}

VectorXd JointConstraint::GetValue(const Eigen::VectorXd &x) const {
    return x.segment<3>(_object_offsets[0] + _point_offsets[0]) - x.segment<3>(_object_offsets[1] + _point_offsets[1]);
}

void JointConstraint::GetGradient(const Eigen::VectorXd &x, COO &coo, int x_offset) const {
    for (int i = 0; i < 3; i++) {
        coo.push_back(Tripletd(x_offset + i, _object_offsets[0] + _point_offsets[0] + i, 1));
        coo.push_back(Tripletd(x_offset + i, _object_offsets[1] + _point_offsets[1] + i, -1));
    }
}