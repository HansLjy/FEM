//
// Created by hansljy on 12/2/22.
//

#ifndef FEM_IPCBARRIERTARGET_H
#define FEM_IPCBARRIERTARGET_H

#include "CollisionAwareTarget.h"

class IPCBarrierTarget : public CollisionAwareTarget {
public:
    IPCBarrierTarget(const Target& target, const ObjectIterator& itr, const json& config)
        : CollisionAwareTarget(target, itr, config) {}

    double GetBarrierEnergy() const override;
    double GetBarrierEnergy(const Ref<const Eigen::VectorXd> &x) const override;
    VectorXd GetBarrierEnergyGradient() const override;
    VectorXd GetBarrierEnergyGradient(const Ref<const Eigen::VectorXd> &x) const override;
    void GetBarrierEnergyHessian(COO &coo, int offset_x, int offset_y) const override;
    void GetBarrierEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const override;

    DERIVED_DECLARE_CLONE(Target)
};

#endif //FEM_IPCBARRIERTARGET_H
