//
// Created by hansljy on 10/8/22.
//

#ifndef FEM_EXTERNALFORCE_H
#define FEM_EXTERNALFORCE_H


#include "Pattern.h"
#include "EigenAll.h"

class Object;

class ExternalForce {
public:
    /**
     * @return The external force in **local** coordinate system
     */
    virtual VectorXd GetExternalForce(const Object& obj, const Matrix3d& rotation, const Vector3d& position) const = 0;
    
    /**
     * @return Vector3d The sum / integration of current force in **local** coordinate system
     */
    virtual Vector3d GetTotalForce(const Object& obj, const Matrix3d& rotation, const Vector3d& position) const = 0;

    /**
     * @return Matrix3d The sum / integration of f * x^T in **local** coordinate system
     */
    virtual Matrix3d GetTotalForceAffineTorque(const Object& obj, const Matrix3d& rotation, const Vector3d& position) const = 0;

    virtual ~ExternalForce() = default;

    BASE_DECLARE_CLONE(ExternalForce)
};

DECLARE_XXX_FACTORY(ExternalForce)

#endif //FEM_EXTERNALFORCE_H
