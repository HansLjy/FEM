//
// Created by hansljy on 10/11/22.
//

#ifndef FEM_TARGET_H
#define FEM_TARGET_H

#include "EigenAll.h"

/**
 * Target is the general target problem encountered
 * in the field of physics-based simulation
 */
class Target {
public:
    virtual VectorXd GetCoordinate() const = 0;
    virtual VectorXd GetVelocity() const = 0;

    virtual void SetCoordinate(const VectorXd& x) = 0;
    virtual void SetVelocity(const VectorXd& v) = 0;

    virtual void GetMass(SparseMatrixXd& mass) const = 0;

    virtual double GetEnergy() const = 0;
    virtual double GetEnergy(const VectorXd& x) const = 0;
    virtual VectorXd GetEnergyGradient() const = 0;
    virtual void GetEnergyHessian(SparseMatrixXd& hessian) const = 0;

    virtual VectorXd GetConstraint(const VectorXd &x) const = 0;
    virtual void GetConstraintGradient(SparseMatrixXd &gradient, const VectorXd &x) const = 0;

    virtual ~Target() = default;
};

#endif //FEM_TARGET_H
