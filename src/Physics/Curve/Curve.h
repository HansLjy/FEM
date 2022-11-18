//
// Created by hansljy on 10/19/22.
//

#ifndef FEM_CURVE_H
#define FEM_CURVE_H

#include "Object.h"

class CurveGravity;

class Curve : public ShapedObject, public SampledObject {
public:
    Curve(double rho, double alpha_max, double alpha_min, const VectorXd &x);

    double GetPotential(const Ref<const VectorXd>& x) const override = 0;
    VectorXd GetPotentialGradient(const Ref<const VectorXd>& x) const override = 0;
    void GetPotentialHessian(const Ref<const VectorXd>& x, COO &coo, int x_offset, int y_offset) const override = 0;

    int GetConstraintSize() const override;
    VectorXd GetInnerConstraint(const Eigen::VectorXd &x) const override;
    void GetInnerConstraintGradient(const Eigen::VectorXd &x, COO &coo, int x_offset, int y_offset) const override;

    MIDDLE_DECLARE_CLONE(Object)

    friend class CurveGravity;

protected:
    VectorXd _alpha;
    int _num_points;            // number of sampled points (end points included)
    VectorXd _x_rest;           // rest shape of the curve
    VectorXd _rest_length;      // length of every edge in the rest shape
    VectorXd _voronoi_length;   // length under the government of one point

    static VectorXd GetX(const Vector3d& start, const Vector3d& end, int num_segments);

private:
    static VectorXd GenerateMass(double rho, const VectorXd &x);
};

#endif //FEM_CURVE_H
