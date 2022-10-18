//
// Created by hansljy on 10/8/22.
//

#include "Object.h"
#include "Curve/Curve.h"
#include "Shape.h"

double Object::GetEnergy() const {
    return GetPotential() + GetExternalEnergy();
}

VectorXd Object::GetEnergyGradient() const {
    return GetPotentialGradient() + GetExternalEnergyGradient();
}

void Object::GetEnergyHessian(COO &coo, int x_offset, int y_offset) const {
    GetPotentialHessian(coo, x_offset, y_offset);
    GetExternalEnergyHessian(coo, x_offset, y_offset);
}

void Object::AddExternalForce(const ExternalForce &force) {
    _external_forces.push_back(force.Clone());
}

double Object::GetExternalEnergy() const {
    double energy = 0;
    for (const auto& ext_force : _external_forces) {
        energy += ext_force->Energy(*this);
    }
    return energy;
}

VectorXd Object::GetExternalEnergyGradient() const {
    VectorXd gradient(this->GetDOF());
    gradient.setZero();
    for (const auto& ext_force : _external_forces) {
        gradient += ext_force->EnergyGradient(*this);
    }
    return gradient;
}

void Object::GetExternalEnergyHessian(COO &coo, int x_offset, int y_offset) const {
    for (const auto& ext_force : _external_forces) {
        ext_force->EnergyHessian(*this, coo, x_offset, y_offset);
    }
}

Object::~Object() {
    for (auto& ext_force : _external_forces) {
        delete ext_force;
    }
    delete _shape;
}

Object::Object(const Object &rhs) {
    _x = rhs._x;
    _v = rhs._v;
    for (const auto& ext_force : rhs._external_forces) {
        _external_forces.push_back(ext_force->Clone());
    }
    _shape = rhs._shape->Clone();
}

Object &Object::operator=(const Object &rhs) {
    if (this == &rhs) {
        return *this;
    }
    _x = rhs._x;
    _v = rhs._v;
    for (const auto& ext_force : _external_forces) {
        delete ext_force;
    }
    _external_forces.clear();
    for (const auto& ext_force : rhs._external_forces) {
        _external_forces.push_back(ext_force->Clone());
    }
    delete _shape;
    _shape = rhs._shape->Clone();
    return *this;
}

BEGIN_DEFINE_XXX_FACTORY(Object)
    ADD_PRODUCT("elastic-curve", Curve)
END_DEFINE_XXX_FACTORY