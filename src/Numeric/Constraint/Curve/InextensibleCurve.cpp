//
// Created by hansljy on 10/11/22.
//

#include "InextensibleCurve.h"
#include "Curve/Curve.h"
#include "spdlog/spdlog.h"

InextensibleCurve::InextensibleCurve(const Curve &curve, int curve_index) :
    Constraint(),
    _size(curve._num_points - 1),
    _curve_index(curve_index),
    _curve_dof(curve.GetDOF()),
    _rest_length(curve._rest_length) {
}

int InextensibleCurve::GetObjectsNum() const {
    return 1;
}

int InextensibleCurve::GetObjectIndex(int object_id) const {
    return _curve_index;
}

void InextensibleCurve::SetOffset(int offset_id, int offset) {
    if (offset_id == 0) {
        _offset = offset;
    } else {
        spdlog::warn("Invalid offset id, offset not set");
    }
}

int InextensibleCurve::GetSize() const {
    return _size;
}

VectorXd InextensibleCurve::GetValue(const VectorXd &x) const {
    const auto& x_curve = x.block(_offset, 0, _curve_dof, 1);
    VectorXd result(_size);
    for (int i = 0, j = 0; i < _size; i++, j += 3) {
        Vector3d e = x_curve.block<3, 1>(j + 3, 0) - x_curve.block<3, 1>(j, 0);
        result(i) = e.dot(e) - _rest_length(i) * _rest_length(i);
    }
    return result;
}

void InextensibleCurve::GetGradient(const VectorXd &x, COO &coo, int x_offset) const {
    int y_offset = _offset;
    const auto& x_curve = x.block(y_offset, 0, _curve_dof, 1);
    for (int i = 0, j = 0; i < _size; i++, j += 3) {
        Vector3d e = x_curve.block<3, 1>(j + 3, 0) - x_curve.block<3, 1>(j, 0);
        for (int k = 0; k < 3; k++) {
            coo.push_back(Tripletd(x_offset + i, y_offset + j + k, -2 * e(k)));
            coo.push_back(Tripletd(x_offset + i, y_offset + j + k + 3, 2 * e(k)));
        }
    }
}

DEFINE_CLONE(Constraint, InextensibleCurve)