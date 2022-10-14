//
// Created by hansljy on 10/11/22.
//

#ifndef FEM_INEXTENSIBLECURVE_H
#define FEM_INEXTENSIBLECURVE_H

#include "Constraint/Constraint.h"
#include "Curve/Curve.h"

class InextensibleCurve final : public Constraint {
public:
    InextensibleCurve(const Curve &curve, int curve_index);

    int GetObjectsNum() const override;
    int GetObjectIndex(int object_id) const override;
    void SetOffset(int offset_id, int offset) override;

    int GetSize() const override;
    VectorXd GetValue(const VectorXd &x) const override;
    void GetGradient(const VectorXd &x, COO &coo, int x_offset) const override;

    DERIVED_DECLARE_CLONE(Constraint)

protected:
    const int _size;
    const int _curve_index;
    const int _curve_dof;
    int _offset;
    const VectorXd _rest_length;
};

#endif //FEM_INEXTENSIBLECURVE_H
