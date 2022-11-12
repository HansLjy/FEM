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

    double Energy(const Object &obj, const Eigen::VectorXd &x, const Eigen::Matrix3d &rotation, const Eigen::Vector3d &position) const override;
    VectorXd EnergyGradient(const Object &obj, const Matrix3d &rotation, const Vector3d &position) const override;
    void EnergyHessian(const Object &obj, const Matrix3d &rotation, const Vector3d &position, COO &coo, int x_offset,
                       int y_offset) const override;

    DERIVED_DECLARE_CLONE(ExternalForce)
protected:
    const Vector3d _g;
};

#endif //FEM_SAMPLEDOBJECTGRAVITY_H
