//
// Created by hansljy on 10/31/22.
//

#ifndef FEM_SAMPLEDOBJECTGRAVITY_H
#define FEM_SAMPLEDOBJECTGRAVITY_H

#include "ExternalForce/ExternalForce.h"

class SampledObjectGravity : public ExternalForce {
public:
    SampledObjectGravity(const json& config);
    SampledObjectGravity(const Vector3d& g);

    VectorXd GetExternalForce(const Object &obj, const Matrix3d &rotation, const Vector3d &position) const override;
    Vector3d GetTotalForce(const Object &obj, const Matrix3d &rotation, const Vector3d &position) const override;
    Matrix3d GetTotalForceAffineTorque(const Object &obj, const Matrix3d &rotation, const Vector3d &position) const override;

    DERIVED_DECLARE_CLONE(ExternalForce)
protected:
    const Vector3d _g;
};

#endif //FEM_SAMPLEDOBJECTGRAVITY_H
