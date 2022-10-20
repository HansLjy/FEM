//
// Created by hansljy on 10/19/22.
//

#ifndef FEM_CURVE_H
#define FEM_CURVE_H

#include "Object.h"

class CurveGravity;

class Curve : public ShapedObject {
public:
    explicit Curve(const json& config);
    Curve(double total_mass, double alpha, const Vector3d &start, const Vector3d &end, int num_segments);
    Curve(double total_mass, double alpha, const VectorXd &x);

    void GetMass(COO &coo, int x_offset, int y_offset) const override;

    double GetPotential() const override = 0;
    VectorXd GetPotentialGradient() const override = 0;
    void GetPotentialHessian(COO &coo, int x_offset, int y_offset) const override = 0;

    int GetConstraintSize() const override;
    VectorXd GetInnerConstraint(const Eigen::VectorXd &x) const override;
    void GetInnerConstraintGradient(const Eigen::VectorXd &x, COO &coo, int x_offset, int y_offset) const override;

    MIDDLE_DECLARE_CLONE(Object)

    friend class CurveGravity;

protected:
    double _alpha;              // elastic coefficient of the curve
    int _num_points;            // number of sampled points (end points included)
    VectorXd _x_rest;           // rest shape of the curve
    VectorXd _mass;             // mass assigned to every sampled points
    VectorXd _mass_sparse;      // mass assigned to every **coordinate**
    VectorXd _rest_length;      // length of every edge in the rest shape
    VectorXd _voronoi_length;   // length under the government of one point

private:
    static VectorXd GetX(const Vector3d& start, const Vector3d& end, int num_segments);
};

#endif //FEM_CURVE_H
