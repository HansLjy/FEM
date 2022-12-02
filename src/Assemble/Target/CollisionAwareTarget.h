//
// Created by hansljy on 12/2/22.
//

#ifndef FEM_COLLISIONAWARETARGET_H
#define FEM_COLLISIONAWARETARGET_H

#include "Target.h"
#include "ObjectIterator.h"
#include "Collision/Culling/CollisionCulling.h"
#include <memory>

class CollisionAwareTarget : public Target {
public:
    CollisionAwareTarget(const Target& target, const CollisionCulling& culling, const ObjectIterator& itr, double d)
        : _target(target.Clone()), _culling(culling.Clone()), _itr(itr.Clone()), _d(d) {}

    int GetDOF() const override {return _target->GetDOF();}
    void GetCoordinate(Ref<Eigen::VectorXd> x) const override { _target->GetCoordinate(x); }
    void GetVelocity(Ref<Eigen::VectorXd> v) const override {_target->GetVelocity(v); }

    void SetCoordinate(const Ref<const Eigen::VectorXd> &x) override {_target->SetCoordinate(x); }
    void SetVelocity(const Ref<const Eigen::VectorXd> &v) override {_target->SetVelocity(v); }

    void GetMass(COO &coo, int offset_x, int offset_y) const override {_target->GetMass(coo, offset_x, offset_y);}

    double GetPotentialEnergy() const override {
        return _target->GetPotentialEnergy() + GetBarrierEnergy();
    }

    double GetPotentialEnergy(const Ref<const Eigen::VectorXd> &x) const override {
        return _target->GetPotentialEnergy(x) + GetBarrierEnergy(x);
    }

    void GetPotentialEnergyGradient(Ref<Eigen::VectorXd> gradient) const override {
        _target->GetPotentialEnergyGradient(gradient);
        gradient += GetBarrierEnergyGradient();
    }

    void GetPotentialEnergyGradient(const Ref<const Eigen::VectorXd> &x, Ref<Eigen::VectorXd> gradient) const override {
        _target->GetPotentialEnergyGradient(x, gradient);
        gradient += GetBarrierEnergyGradient(x);
    }

    void GetPotentialEnergyHessian(COO &coo, int offset_x, int offset_y) const override {
        _target->GetPotentialEnergyHessian(coo, offset_x, offset_y);
        GetBarrierEnergyHessian(coo, offset_x, offset_y);
    }

    void GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const override {
        _target->GetPotentialEnergyHessian(x, coo, offset_x, offset_y);
        GetBarrierEnergyHessian(x, coo, offset_x, offset_y);
    }

    void GetExternalForce(Ref<Eigen::VectorXd> force) const override { _target->GetExternalForce(force); }

    VectorXd GetConstraint(const Eigen::VectorXd &x) const override {return _target->GetConstraint(x); }
    void GetConstraintGradient(SparseMatrixXd &gradient, const Eigen::VectorXd &x) const override { _target->GetConstraintGradient(gradient, x); }

    void UpdateInfo(const Eigen::VectorXd &x, int time_stamp) override;

    ~CollisionAwareTarget() override {
        delete _target;
        delete _culling;
    }

    CollisionAwareTarget(const CollisionAwareTarget& rhs)
        : _target(rhs._target->Clone()), _culling(rhs._culling->Clone()), _itr(rhs._itr), _collision_info(rhs._collision_info), _d(rhs._d) {}

    MIDDLE_DECLARE_CLONE(Target)

protected:
    virtual double GetBarrierEnergy() const = 0;
    virtual double GetBarrierEnergy(const Ref<const VectorXd>& x) const = 0;
    virtual VectorXd GetBarrierEnergyGradient() const = 0;
    virtual VectorXd GetBarrierEnergyGradient(const Ref<const VectorXd>& x) const = 0;
    virtual void GetBarrierEnergyHessian(COO& coo, int offset_x, int offset_y) const = 0;
    virtual void GetBarrierEnergyHessian(const Ref<const VectorXd>& x, COO& coo, int offset_x, int offset_y) const = 0;

protected:
    Target* _target;
    CollisionCulling* _culling;
    std::shared_ptr<const ObjectIterator> _itr;
    std::vector<CollisionInfo> _collision_info;
    double _d;
};

#endif //FEM_COLLISIONAWARETARGET_H
