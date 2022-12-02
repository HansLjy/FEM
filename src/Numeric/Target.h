//
// Created by hansljy on 10/11/22.
//

#ifndef FEM_TARGET_H
#define FEM_TARGET_H

#include "EigenAll.h"
#include "Pattern.h"

/**
 * Target is the general target problem encountered
 * in the field of physics-based simulation
 */
class Target {
public:
    virtual int GetDOF() const = 0;

    virtual void GetCoordinate(Ref<VectorXd> x) const = 0;
    virtual void GetVelocity(Ref<VectorXd> v) const = 0;

    virtual void SetCoordinate(const Ref<const VectorXd> &x) = 0;
    virtual void SetVelocity(const Ref<const VectorXd> &v) = 0;

    void GetMass(SparseMatrixXd& mass) const {
        COO coo;
        GetMass(coo, 0, 0);
        mass.resize(GetDOF(), GetDOF());
        mass.setFromTriplets(coo.begin(), coo.end());
    }
    virtual void GetMass(COO& coo, int offset_x, int offset_y) const = 0;

    virtual double GetPotentialEnergy() const = 0;
    virtual double GetPotentialEnergy(const Ref<const VectorXd>& x) const = 0;
    virtual void GetPotentialEnergyGradient(Ref<VectorXd> gradient) const = 0;
    virtual void GetPotentialEnergyGradient(const Ref<const VectorXd> &x, Ref<VectorXd> gradient) const = 0;
    void GetPotentialEnergyHessian(SparseMatrixXd& hessian) const {
        COO coo;
        GetPotentialEnergyHessian(coo, 0, 0);
        hessian.resize(GetDOF(), GetDOF());
        hessian.setFromTriplets(coo.begin(), coo.end());
    }
    virtual void GetPotentialEnergyHessian(COO& coo, int offset_x, int offset_y) const = 0;
    void GetPotentialEnergyHessian(const Ref<const VectorXd>& x, SparseMatrixXd& hessian) const {
        COO coo;
        GetPotentialEnergyHessian(x, coo, 0, 0);
        hessian.resize(GetDOF(), GetDOF());
        hessian.setFromTriplets(coo.begin(), coo.end());
    }
    virtual void GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const = 0;
    virtual void GetExternalForce(Ref<VectorXd> force) const = 0;

    virtual VectorXd GetConstraint(const VectorXd &x) const = 0;
    virtual void GetConstraintGradient(SparseMatrixXd &gradient, const VectorXd &x) const = 0;

    virtual void UpdateInfo(const VectorXd& x, int time_stamp) {}

    virtual ~Target() = default;

    BASE_DECLARE_CLONE(Target)
};

#endif //FEM_TARGET_H
