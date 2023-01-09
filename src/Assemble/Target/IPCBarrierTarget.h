//
// Created by hansljy on 12/2/22.
//

#ifndef FEM_IPCBARRIERTARGET_H
#define FEM_IPCBARRIERTARGET_H

#include "Collision/Culling/CollisionCulling.h"
#include "Collision/CCD/CCD.h"
#include "EigenAll.h"
#include "Object.h"
#include "Target.h"
#include <Eigen/src/Core/Matrix.h>
#include <vector>

class CollisionCulling;

class IPCBarrierTarget : public Target {
public:
    IPCBarrierTarget(const std::vector<Object*>& objs, int begin, int end, const json& config);

    /**
     * @note This function will actually compute the collision shape of
     *       every object in the system, so you don't need to call other
     *       function to do the job
     */
    void ComputeConstraintSet(const Eigen::VectorXd &x, int time_stamp);

    double GetPotentialEnergy(const Ref<const Eigen::VectorXd> &x) const override {
        return Target::GetPotentialEnergy(x) + GetBarrierEnergy();
    }

    void GetPotentialEnergyGradient(const Ref<const Eigen::VectorXd> &x, Ref<Eigen::VectorXd> gradient) const override {
        Target::GetPotentialEnergyGradient(x, gradient);
        gradient += GetBarrierEnergyGradient();
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

    double GetVFBarrierEnergy(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    double GetEEBarrierEnergy(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;
    Vector12d GetVFBarrierEnergyGradient(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    Vector12d GetEEBarrierEnergyGradient(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;
    Matrix12d GetVFBarrierEnergyHessian(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) const;
    Matrix12d GetEEBarrierEnergyHessian(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) const;

    double _d_hat;
    std::vector<CollisionInfo> _constraint_set;
    CollisionCulling* _culling;
    CCD* _ccd;
};

#endif //FEM_IPCBARRIERTARGET_H
