//
// Created by hansljy on 12/2/22.
//

#include "IPCBarrierTarget.h"

DEFINE_CLONE(Target, IPCBarrierTarget)

double IPCBarrierTarget::GetBarrierEnergy() const {
    return 0;
    // TODO
}

double IPCBarrierTarget::GetBarrierEnergy(const Ref<const Eigen::VectorXd> &x) const {
    return 0;
    // TODO
}

VectorXd IPCBarrierTarget::GetBarrierEnergyGradient() const {
    VectorXd gradient(CollisionAwareTarget::GetDOF());
    gradient.setZero();
    return gradient;
    // TODO
}

VectorXd IPCBarrierTarget::GetBarrierEnergyGradient(const Ref<const Eigen::VectorXd> &x) const {
    VectorXd gradient(CollisionAwareTarget::GetDOF());
    gradient.setZero();
    return gradient;
    // TODO
}

void IPCBarrierTarget::GetBarrierEnergyHessian(COO &coo, int offset_x, int offset_y) const {
    // TODO
}

void IPCBarrierTarget::GetBarrierEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x,
                                               int offset_y) const {
    // TODO
}