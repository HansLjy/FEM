//
// Created by hansljy on 10/19/22.
//

#ifndef FEM_CURVE_H
#define FEM_CURVE_H

#include "Object.h"

class CurveGravity;

class Curve : public SampledObject {
public:
	Curve(const json& config);
	Curve(bool collision_enabled, double rho, double alpha_max, double alpha_min, const Vector3d &start, const Vector3d &end, int num_segments) : Curve(collision_enabled, rho, alpha_max, alpha_min, GetX(start, end, num_segments)) {}
    Curve(bool collision_enabled, double rho, double alpha_max, double alpha_min, const VectorXd &x);

	double GetPotential(const Ref<const VectorXd> &x) const override;
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override;
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override;

    DERIVED_DECLARE_CLONE(Object)

    friend class CurveGravity;

protected:
    const double _k;      		// stiffness of the curve
    int _num_points;            // number of sampled points (end points included)
    VectorXd _alpha;
    VectorXd _x_rest;           // rest shape of the curve
    VectorXd _rest_length;      // length of every edge in the rest shape
    VectorXd _voronoi_length;   // length under the government of one point

    static VectorXd GetX(const Vector3d& start, const Vector3d& end, int num_segments);
    static VectorXd GenerateMass(double rho, const VectorXd &x);
};

#endif //FEM_CURVE_H
