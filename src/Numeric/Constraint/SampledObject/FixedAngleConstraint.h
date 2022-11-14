//
// Created by hansljy on 11/14/22.
//

#ifndef FEM_FIXEDANGLECONSTRAINT_H
#define FEM_FIXEDANGLECONSTRAINT_H

#include "Constraint/Constraint.h"

class FixedAngleConstraint : public Constraint {
public:
    // point2 - point1 || direction
    FixedAngleConstraint(const Vector3d& direction, int object_id, int point_id1, int point_id2);

    VectorXd GetValue(const Eigen::VectorXd &x) const override;
    void GetGradient(const Eigen::VectorXd &x, COO &coo, int x_offset) const override;

    DERIVED_DECLARE_CLONE(Constraint)

protected:
    const int _point_offset1;
    const int _point_offset2;
    const Vector3d _direction;
};

#endif //FEM_FIXEDANGLECONSTRAINT_H
