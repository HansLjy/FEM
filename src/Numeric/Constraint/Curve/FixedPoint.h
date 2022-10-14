//
// Created by hansljy on 10/11/22.
//

#ifndef FEM_FIXPOINT_H
#define FEM_FIXPOINT_H

#include "Constraint/Constraint.h"
#include "Curve/Curve.h"
#include "nlohmann/json.hpp"

class FixedPoint final : public Constraint {
public:
    FixedPoint(const Curve& curve, int curve_idx, int fixed_point_idx, const Vector3d& fixed_point);

    int GetSize() const override;
    int GetObjectsNum() const override;
    int GetObjectIndex(int object_id) const override;
    void SetOffset(int offset_id, int offset) override;
    VectorXd GetValue(const Eigen::VectorXd &x) const override;
    void GetGradient(const Eigen::VectorXd &x, COO &coo, int x_offset) const override;

    DERIVED_DECLARE_CLONE(Constraint)

private:
    const int _curve_idx;
    const int _fixed_point_idx;
    int _curve_offset;
    Vector3d _fixed_point;
};

#endif //FEM_FIXPOINT_H
