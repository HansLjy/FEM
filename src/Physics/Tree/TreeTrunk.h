//
// Created by hansljy on 11/10/22.
//

#ifndef FEM_TREETRUNK_H
#define FEM_TREETRUNK_H

#include "ReducedObject/ReducedBezierCurve.h"

class TreeDomain;

class TreeTrunk : public ReducedBezierCurve {
public:
    explicit TreeTrunk(const json& json);
    TreeTrunk(int num_segments, double rho, double alpha_max, double alpha_min,
              const VectorXd &control_points, const Vector3d& x_root);

    friend class TreeDomain;
    DERIVED_DECLARE_CLONE(Object)
protected:
    const Vector3d _x_root; // every tree trunk has a root, which is supposed
                            // to be in fixed position in the material frame
                            // the root will move along with the parent component
};

#endif //FEM_TREETRUNK_H
