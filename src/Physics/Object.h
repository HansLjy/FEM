//
// Created by hansljy on 10/7/22.
//

#ifndef FEM_OBJECT_H
#define FEM_OBJECT_H

#include "EigenAll.h"
#include "Pattern.h"
#include "ExternalForce/ExternalForce.h"

class Shape;

class Object {
public:

    int GetDOF() const {
        return _x.size();
    }

    const VectorXd & GetCoordinate() const {
        return _x;
    }

    const VectorXd & GetVelocity() const {
        return _v;
    }

    VectorXd& GetCoordinate() {
        return _x;
    }

    VectorXd& GetVelocity() {
        return _v;
    }

    /**
     * The mass should satisfy the definition of kinetic energy
     * T = 1/2 v^T M v
     */
    virtual void GetMass(SparseMatrixXd &mass) const = 0;  // General mass (sparse matrix form)
    virtual void GetMass(COO &coo, int x_offset, int y_offset) const = 0;           // General mass (coo form)

    double GetEnergy() const;
    VectorXd GetEnergyGradient() const;
    void GetEnergyHessian(SparseMatrixXd& hessian) const;
    void GetEnergyHessian(COO& coo, int x_offset, int y_offset) const;

    virtual double GetPotential() const = 0;
    virtual VectorXd GetPotentialGradient() const = 0;
    void GetPotentialHessian(SparseMatrixXd &hessian) const;
    virtual void GetPotentialHessian(COO &coo, int x_offset, int y_offset) const = 0;

    void AddExternalForce(const ExternalForce& force);

    double GetExternalEnergy() const;
    VectorXd GetExternalEnergyGradient() const;
    void GetExternalEnergyHessian(SparseMatrixXd& hessian) const;
    void GetExternalEnergyHessian(COO& coo, int x_offset, int y_offset) const;

    const Shape * GetShape() {
        return _shape;
    }

    virtual ~Object();
    Object() = default;
    Object(const Object& rhs);
    Object& operator=(const Object& rhs);

    BASE_DECLARE_CLONE(Object)

protected:
    VectorXd _x;
    VectorXd _v;

    std::vector<const ExternalForce*> _external_forces;
    Shape* _shape;
};

DECLARE_XXX_FACTORY(Object)

#endif //FEM_OBJECT_H
