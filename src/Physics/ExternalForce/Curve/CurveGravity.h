//
// Created by hansljy on 10/11/22.
//

#ifndef FEM_CURVEGRAVITY_H
#define FEM_CURVEGRAVITY_H

#include "ExternalForce/ExternalForce.h"

class CurveGravity : public ExternalForce {
public:
    CurveGravity(const json& config);

    //-> the length of g stand for the gravity acceleration
    CurveGravity(const Vector3d& g);

    double Energy(const Object &obj) const override;
    VectorXd EnergyGradient(const Object& obj) const override;
    void EnergyHessian(const Object &obj, COO &coo, int x_offset, int y_offset) const override;
    void EnergyHessian(const Object &obj, SparseMatrixXd &hessian) const override;

    DERIVED_DECLARE_CLONE(ExternalForce)
protected:
    Vector3d _g;
};

#endif //FEM_SAMPLEOBJECTGRAVITY_H
