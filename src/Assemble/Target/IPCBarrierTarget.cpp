//
// Created by hansljy on 12/2/22.
//

#include "IPCBarrierTarget.h"

void IPCBarrierTarget::UpdateInfo(const Eigen::VectorXd &x, int time_stamp) {
    // TODO
}

double IPCBarrierTarget::GetBarrierEnergy() const {
    return 0;
    // TODO
}

VectorXd IPCBarrierTarget::GetBarrierEnergyGradient() const {
    VectorXd gradient(GetDOF());
    gradient.setZero();
    return gradient;
    // TODO
}

void IPCBarrierTarget::GetBarrierEnergyHessian(COO &coo, int offset_x, int offset_y) const {
    // TODO
}

double IPCBarrierTarget::GetMaxStep(const Eigen::VectorXd &p) {
    return 1;
    // TODO
}