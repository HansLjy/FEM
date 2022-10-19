//
// Created by hansljy on 10/18/22.
//

#include "ReducedObject.h"

ReducedObject::ReducedObject(const VectorXd &x, const Object &proxy, const SparseMatrixXd &base)
    : Object(x), _proxy(proxy.Clone()), _base(base) {}

void ReducedObject::SetCoordinate(const Eigen::VectorXd &x) {
    Object::SetCoordinate(x);
    SetProxyCoordinate();
}

void ReducedObject::SetVelocity(const Eigen::VectorXd &v) {
    Object::SetVelocity(v);
    SetProxyVelocity();
}

void ReducedObject::SetProxyCoordinate() {
    _proxy->SetCoordinate(_base * _x);
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

void ReducedObject::GetMass(COO &coo, int x_offset, int y_offset) const {
    LUMP2D(Mass, mass)
}

double ReducedObject::GetPotential() const {
    return _proxy->GetPotential();
}

VectorXd ReducedObject::GetPotentialGradient() const {
    return _base.transpose() * _proxy->GetPotentialGradient();
}

void ReducedObject::GetPotentialHessian(COO &coo, int x_offset, int y_offset) const {
    LUMP2D(PotentialHessian, hessian)
}

void ReducedObject::AddExternalForce(const ExternalForce &force) {
    _proxy->AddExternalForce(force);
}

double ReducedObject::GetExternalEnergy() const {
    return _proxy->GetExternalEnergy();
}

VectorXd ReducedObject::GetExternalEnergyGradient() const {
    return _base.transpose() * _proxy->GetExternalEnergyGradient();
}

void ReducedObject::GetExternalEnergyHessian(COO &coo, int x_offset, int y_offset) const {
    LUMP2D(ExternalEnergyHessian, hessian)
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

const void ReducedObject::GetShape(Eigen::MatrixXd &vertices, Eigen::MatrixXi &topo) const {
    _proxy->GetShape(vertices, topo);
}

ReducedObject::~ReducedObject() {
    delete _proxy;
}

ReducedObject::ReducedObject(const ReducedObject &rhs)  : Object(rhs) {
    _proxy = rhs._proxy->Clone();
    _base = rhs._base;
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