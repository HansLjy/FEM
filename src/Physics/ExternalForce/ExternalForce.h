//
// Created by hansljy on 10/8/22.
//

#ifndef FEM_EXTERNALFORCE_H
#define FEM_EXTERNALFORCE_H


#include "Pattern.h"
#include "EigenAll.h"

class Object;

/**
 * Can only handle conservative force
 */
class ExternalForce {
public:
    virtual double Energy(const Object& obj) const = 0;
    virtual VectorXd EnergyGradient(const Object& obj) const = 0;

    virtual void EnergyHessian(const Object& obj, COO& coo, int x_offset, int y_offset) const = 0;

    virtual ~ExternalForce() = default;

    BASE_DECLARE_CLONE(ExternalForce)
};

DECLARE_XXX_FACTORY(ExternalForce)

#endif //FEM_EXTERNALFORCE_H
