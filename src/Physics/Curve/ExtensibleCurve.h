//
// Created by hansljy on 10/19/22.
//

#ifndef FEM_EXTENSIBLECURVE_H
#define FEM_EXTENSIBLECURVE_H

#include "Curve.h"

class ExtensibleCurve : public Curve {
public:
    explicit ExtensibleCurve(const json& config) : Curve(config), _k(1000 * _alpha * (_num_points - 1)) {};
    ExtensibleCurve(double total_mass, double alpha, const Vector3d& start, const Vector3d& end, int num_segments)
        : Curve(total_mass, alpha, start, end, num_segments), _k(1000 * _alpha * (_num_points - 1)) {};
    ExtensibleCurve(double total_mass, double alpha, const VectorXd& x)
        : Curve(total_mass, alpha, x), _k(1000 * _alpha * (_num_points - 1)) {};

    double GetPotential() const override;
    VectorXd GetPotentialGradient() const override;
    void GetPotentialHessian(COO &coo, int x_offset, int y_offset) const override;

    DERIVED_DECLARE_CLONE(Object)

protected:
    const double _k;      // stiffness of the curve
};

#endif //FEM_EXTENSIBLECURVE_H