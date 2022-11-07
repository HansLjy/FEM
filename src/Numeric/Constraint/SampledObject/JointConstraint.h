//
// Created by hansljy on 11/2/22.
//

#ifndef FEM_JOINTCONSTRAINT_H
#define FEM_JOINTCONSTRAINT_H

#include "Constraint/Constraint.h"

class JointConstraint : public Constraint {
public:
    JointConstraint(int object1_id, int point1_id, int object2_id, int point2_id);
    VectorXd GetValue(const Eigen::VectorXd &x) const override;
    void GetGradient(const Eigen::VectorXd &x, COO &coo, int x_offset) const override;

    DERIVED_DECLARE_CLONE(Constraint)
protected:
    std::array<int, 2> _point_offsets;
};

#endif //FEM_JOINTCONSTRAINT_H
