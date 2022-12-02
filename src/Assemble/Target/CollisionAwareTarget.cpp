//
// Created by hansljy on 12/2/22.
//

#include "CollisionAwareTarget.h"

void CollisionAwareTarget::UpdateInfo(const Eigen::VectorXd &x, int time_stamp) {
    _culling->ComputeConstraintSet(x, _itr, time_stamp, _d, _collision_info);
}