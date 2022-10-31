//
// Created by hansljy on 10/19/22.
//

#ifndef FEM_EXTENSIBLECURVE_H
#define FEM_EXTENSIBLECURVE_H

#include "Curve.h"

class ExtensibleCurve : public Curve {
public:
    explicit ExtensibleCurve(const json& config);
    ExtensibleCurve(double rho, double alpha_max, double alpha_min, const Vector3d &start, const Vector3d &end,
                    int num_segments)
        : ExtensibleCurve(rho, alpha_max, alpha_min, Curve::GetX(start, end, num_segments)) {};
    ExtensibleCurve(double rho, double alpha_max, double alpha_min, const VectorXd &x)
        : Object(x), Curve(rho, alpha_max, alpha_min, x), _k(100 * alpha_min) {};

    double GetPotential() const override;
    VectorXd GetPotentialGradient() const override;
    void GetPotentialHessian(COO &coo, int x_offset, int y_offset) const override;

    DERIVED_DECLARE_CLONE(Object)

protected:
    const double _k;      // stiffness of the curve
};

#endif //FEM_EXTENSIBLECURVE_H
