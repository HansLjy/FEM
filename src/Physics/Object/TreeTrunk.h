//
// Created by hansljy on 11/11/22.
//

#ifndef FEM_TREETRUNK_H
#define FEM_TREETRUNK_H

#include "Object.h"

class TreeTrunkShape;

class TreeTrunk : public SampledObject {
public:
    TreeTrunk(bool collision_enabled, double rho, double youngs_module, double radius_max, double radius_min, const VectorXd &x, const Vector3d &root);
    double GetPotential(const Ref<const VectorXd>& x) const override;
    VectorXd GetPotentialGradient(const Ref<const VectorXd>& x) const override;
    void GetPotentialHessian(const Ref<const VectorXd>& x, COO& coo, int x_offset, int y_offset) const override;

    friend class TreeTrunkShape;

protected:
    const Vector3d _root;
    VectorXd _stiffness;
    VectorXd _alpha;
    VectorXd _x_rest;           // rest shape of the curve
    VectorXd _rest_length;      // length of every edge in the rest shape
    VectorXd _voronoi_length;   // length under the government of one point

private:
    static VectorXd GenerateMass(const VectorXd& x, double rho, double radius_max, double radius_min);
	static MatrixXi GenerateTopo(int n);
};

#endif //FEM_TREETRUNK_H
