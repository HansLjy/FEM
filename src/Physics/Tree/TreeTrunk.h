//
// Created by hansljy on 11/11/22.
//

#ifndef FEM_TREETRUNK_H
#define FEM_TREETRUNK_H

#include "Curve/ExtensibleCurve.h"

class TreeTrunkGravity;
class TreeTrunkShape;

class TreeTrunk : public SampledObject, public ShapedObject {
public:
    TreeTrunk(double rho, double alpha_max, double alpha_min, double radius_max, double radius_min, double k, const VectorXd &x, const Vector3d& root);
    double GetPotential() const override;
    VectorXd GetPotentialGradient() const override;
    void GetPotentialHessian(COO &coo, int x_offset, int y_offset) const override;

    DERIVED_DECLARE_CLONE(Object)

    friend class TreeTrunkGravity;
    friend class TreeTrunkShape;

protected:
    const int _num_points;            // number of sampled points (end points included)
    const double _k;
    const Vector3d _root;
    VectorXd _alpha;
    VectorXd _x_rest;           // rest shape of the curve
    VectorXd _rest_length;      // length of every edge in the rest shape
    VectorXd _voronoi_length;   // length under the government of one point

private:
    static VectorXd GenerateMass(const VectorXd& x, double rho);
};

#endif //FEM_TREETRUNK_H
