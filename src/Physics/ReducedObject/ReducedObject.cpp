//
// Created by hansljy on 10/18/22.
//

#include "ReducedObject.h"

ReducedObject::ReducedObject(const VectorXd &x, const Object &proxy, const SparseMatrixXd &base, const VectorXd &shift)
    : Object(x), _proxy(proxy.Clone()), _base(base), _shift(shift) {}

void ReducedObject::SetCoordinate(const Eigen::VectorXd &x) {
    Object::SetCoordinate(x);
    SetProxyCoordinate();
}

void ReducedObject::SetVelocity(const Eigen::VectorXd &v) {
    Object::SetVelocity(v);
    SetProxyVelocity();
}

void ReducedObject::SetProxyCoordinate() {
    _proxy->SetCoordinate(_base * _x + _shift);
}

void ReducedObject::SetProxyVelocity() {
    _proxy->SetVelocity(_base * _v);
}

#define LUMP2D(FuncName, VarName) \
    COO coo_full;    \
    _proxy->Get##FuncName(coo_full, 0, 0); \
    SparseMatrixXd VarName(_proxy->GetDOF(), _proxy->GetDOF()); \
    VarName.setFromTriplets(coo_full.begin(), coo_full.end()); \
    SparseMatrixXd VarName##_reduced = _base.transpose() * VarName * _base; \
    for (int i = 0; i < VarName##_reduced.outerSize(); ++i) { \
        for (SparseMatrixXd::InnerIterator it(VarName##_reduced, i); it; ++it) { \
            coo.push_back(Tripletd(it.row() + x_offset, it.col() + y_offset, it.value())); \
        } \
    }

#define LUMP2DElseWhere(FuncName, VarName)\
    COO coo_full;    \
    _proxy->Get##FuncName(_base * x + _shift, coo_full, 0, 0); \
    SparseMatrixXd VarName(_proxy->GetDOF(), _proxy->GetDOF()); \
    VarName.setFromTriplets(coo_full.begin(), coo_full.end()); \
    SparseMatrixXd VarName##_reduced = _base.transpose() * VarName * _base; \
    for (int i = 0; i < VarName##_reduced.outerSize(); ++i) { \
        for (SparseMatrixXd::InnerIterator it(VarName##_reduced, i); it; ++it) { \
            coo.push_back(Tripletd(it.row() + x_offset, it.col() + y_offset, it.value())); \
        } \
    }

#define LUMP2DWithInfo(FuncName, VarName) \
    COO coo_full;    \
    _proxy->Get##FuncName(rotation, position, coo_full, 0, 0); \
    SparseMatrixXd VarName(_proxy->GetDOF(), _proxy->GetDOF()); \
    VarName.setFromTriplets(coo_full.begin(), coo_full.end()); \
    SparseMatrixXd VarName##_reduced = _base.transpose() * VarName * _base; \
    for (int i = 0; i < VarName##_reduced.outerSize(); ++i) { \
        for (SparseMatrixXd::InnerIterator it(VarName##_reduced, i); it; ++it) { \
            coo.push_back(Tripletd(it.row() + x_offset, it.col() + y_offset, it.value())); \
        } \
    }

void ReducedObject::GetMass(COO &coo, int x_offset, int y_offset) const {
    LUMP2D(Mass, mass)
}

double ReducedObject::GetTotalMass() const {
    return _proxy->GetTotalMass();
}

double ReducedObject::GetPotential() const {
    return _proxy->GetPotential();
}

double ReducedObject::GetPotential(const Ref<const Eigen::VectorXd> &x) const {
    return _proxy->GetPotential(_base * x + _shift);
}

VectorXd ReducedObject::GetPotentialGradient() const {
    return _base.transpose() * _proxy->GetPotentialGradient();
}

VectorXd ReducedObject::GetPotentialGradient(const Ref<const Eigen::VectorXd> &x) const {
    return _base.transpose() * _proxy->GetPotentialGradient(_base * x + _shift);
}

void ReducedObject::GetPotentialHessian(COO &coo, int x_offset, int y_offset) const {
    LUMP2D(PotentialHessian, hessian)
}

void
ReducedObject::GetPotentialHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
    LUMP2DElseWhere(PotentialHessian, hessian)
}

void ReducedObject::AddExternalForce(const ExternalForce &force) {
    _proxy->AddExternalForce(force);
}

Vector3d ReducedObject::GetTotalExternalForce(const Matrix3d &rotation, const Vector3d &position) const {
    return _proxy->GetTotalExternalForce(rotation, position);
}

VectorXd
ReducedObject::GetInertialForce(const Eigen::Vector3d &v, const Eigen::Vector3d &a, const Eigen::Vector3d &omega,
                                const Eigen::Vector3d &alpha, const Eigen::Matrix3d &rotation) const {
    return _base.transpose() * _proxy->GetInertialForce(v, a, omega, alpha, rotation);
}

int ReducedObject::GetConstraintSize() const {
    return _proxy->GetConstraintSize();
}

VectorXd ReducedObject::GetInnerConstraint(const VectorXd &x) const {
    return _proxy->GetInnerConstraint(_base * x);
}

void ReducedObject::GetInnerConstraintGradient(const Eigen::VectorXd &x, COO &coo, int x_offset, int y_offset) const {
    COO coo_proxy;
    _proxy->GetInnerConstraintGradient(_base * x, coo_proxy, 0, 0);
    SparseMatrixXd gradient( _proxy->GetConstraintSize(), _proxy->GetDOF());
    gradient.setFromTriplets(coo_proxy.begin(), coo_proxy.end());
    gradient = gradient * _base;
    for (int i = 0; i < gradient.outerSize(); i++) {
        for (SparseMatrixXd::InnerIterator it(gradient, i); it; ++it) {
            coo.push_back(Tripletd(it.row() + x_offset, it.col() + y_offset, it.value()));
        }
    }
}

void ReducedObject::GetShape(Eigen::MatrixXd &vertices, Eigen::MatrixXi &topo) const {
    _proxy->GetShape(vertices, topo);
}

ReducedObject::~ReducedObject() {
    delete _proxy;
}

ReducedObject::ReducedObject(const ReducedObject &rhs)  : Object(rhs) {
    _proxy = rhs._proxy->Clone();
    _base = rhs._base;
    _shift = rhs._shift;
}

ReducedObject &ReducedObject::operator=(const ReducedObject &rhs) {
    if (this == &rhs) {
        return *this;
    }
    delete _proxy;
    _proxy = rhs._proxy->Clone();
    _base = rhs._base;
    return *this;
}