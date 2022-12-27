//
// Created by hansljy on 11/11/22.
//

#ifndef FEM_REDUCEDTREETRUNK_H
#define FEM_REDUCEDTREETRUNK_H

#include "Object.h"

class DecomposedTreeTrunk;

class ReducedTreeTrunk : public ReducedObject {
public:
    explicit ReducedTreeTrunk(const json& config);
    ReducedTreeTrunk(int num_segments, double rho, double youngs_module, double radius_max, double radius_min,
                     const Vector3d &root, const VectorXd &control_points);

    DERIVED_DECLARE_CLONE(Object)
    friend class DecomposedTreeTrunk;

protected:
    const Vector3d _x_root;
    static VectorXd GenerateX(int num_segments, const VectorXd &control_points);
    static SparseMatrixXd GenerateBase(int num_segments);
    static VectorXd GenerateShift(int num_segments, const Vector3d& first_control_point);
};

#endif //FEM_REDUCEDTREETRUNK_H
