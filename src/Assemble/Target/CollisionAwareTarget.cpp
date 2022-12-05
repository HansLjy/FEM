//
// Created by hansljy on 12/2/22.
//

#include "CollisionAwareTarget.h"

void CollisionAwareTarget::UpdateInfo(const Eigen::VectorXd &x, int time_stamp) {
    _culling->ComputeConstraintSet(x, _itr, time_stamp, _d, _collision_info);
}

CollisionAwareTarget::CollisionAwareTarget(const Target &target, const ObjectIterator &itr,
                                           const nlohmann::json &config)
                                           : _target(target.Clone()), _itr(itr.Clone()), _d(config["d"]){
    const auto& culling_config = config["culling"];
    _culling = CollisionCullingFactory::GetCollisionCulling(culling_config["type"], culling_config);
}

#include "IPCBarrierTarget.h"

CollisionAwareTarget *
CollisionAwareTargetFactory::GetCollisionAwareTarget(const Target &target, const ObjectIterator &itr,
                                                     const nlohmann::json &config) {
    std::string type(config["type"]);
    if (type == "ipc-barrier") {
        return new IPCBarrierTarget(target, itr, config);
    } else {
        return nullptr;
    }
}