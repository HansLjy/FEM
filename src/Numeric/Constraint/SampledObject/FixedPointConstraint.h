//
// Created by hansljy on 10/19/22.
//

#ifndef FEM_FIXEDPOINTCONSTRAINT_H
#define FEM_FIXEDPOINTCONSTRAINT_H

#include "Constraint/Constraint.h"
#include "EigenAll.h"

class FixedPointConstraint : public Constraint {
public:
    FixedPointConstraint(const Vector3d& fixed_point, int object_id, int point_id);
    VectorXd GetValue(const VectorXd &x) const override;
    void GetGradient(const VectorXd &x, COO &coo, int x_offset) const override;
	
protected:
    int _point_offset;
    Vector3d _fixed_point;
};

#endif //FEM_FIXEDPOINTCONSTRAINT_H
