//
// Created by hansljy on 10/7/22.
//

#ifndef FEM_INEXTENSIBLECURVE_H
#define FEM_INEXTENSIBLECURVE_H

#include "Object.h"
#include "EigenAll.h"
#include "Curve.h"

class InextensibleCurve : public Curve {
public:
    explicit InextensibleCurve(const json& config) : Curve(config) {};
    InextensibleCurve(double total_mass, double alpha_max, double alpha_min, const Vector3d &start, const Vector3d &end, int num_segments)
        : Curve(total_mass, alpha_max, alpha_min, start, end, num_segments) {};
    InextensibleCurve(double total_mass, double alpha_max, double alpha_min, const VectorXd &x)
        : Curve(total_mass, alpha_max, alpha_min, x) {};

    double GetPotential() const override;
    VectorXd GetPotentialGradient() const override;
    void GetPotentialHessian(COO &coo, int x_offset, int y_offset) const override;

    int GetConstraintSize() const override;
    VectorXd GetInnerConstraint(const VectorXd &x) const override;
    void GetInnerConstraintGradient(const VectorXd &x, COO &coo, int x_offset, int y_offset) const override;

    ~InextensibleCurve() override = default;

    DERIVED_DECLARE_CLONE(Object)
};

#endif //FEM_INEXTENSIBLECURVE_H
