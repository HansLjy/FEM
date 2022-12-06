//
// Created by hansljy on 12/2/22.
//

#ifndef FEM_IPCBARRIERTARGET_H
#define FEM_IPCBARRIERTARGET_H

#include "Target.h"

class IPCBarrierTarget : public Target {
public:
    IPCBarrierTarget(const ObjectIterator& itr)
        : Target(itr) {}

    void UpdateInfo(const Eigen::VectorXd &x, int time_stamp) override;

    double GetPotentialEnergy() const override {
        return Target::GetPotentialEnergy() + GetBarrierEnergy();
    }

    double GetPotentialEnergy(const Ref<const Eigen::VectorXd> &x) const override {
        return Target::GetPotentialEnergy(x) + GetBarrierEnergy();
    }

    void GetPotentialEnergyGradient(Ref<Eigen::VectorXd> gradient) const override {
        Target::GetPotentialEnergyGradient(gradient);
        gradient += GetBarrierEnergyGradient();
    }

    void GetPotentialEnergyGradient(const Ref<const Eigen::VectorXd> &x, Ref<Eigen::VectorXd> gradient) const override {
        Target::GetPotentialEnergyGradient(x, gradient);
        gradient += GetBarrierEnergyGradient();
    }

    void GetPotentialEnergyHessian(COO &coo, int offset_x, int offset_y) const override {
        Target::GetPotentialEnergyHessian(coo, offset_x, offset_y);
        GetBarrierEnergyHessian(coo, offset_x, offset_y);
    }

    void GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const override {
        Target::GetPotentialEnergyHessian(x, coo, offset_x, offset_y);
        GetBarrierEnergyHessian(coo, offset_x, offset_y);
    }

    double GetMaxStep(const VectorXd& p);

protected:
    double GetBarrierEnergy() const;
    VectorXd GetBarrierEnergyGradient() const;
    void GetBarrierEnergyHessian(COO &coo, int offset_x, int offset_y) const;

};

#endif //FEM_IPCBARRIERTARGET_H
