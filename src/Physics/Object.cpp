//
// Created by hansljy on 10/8/22.
//

#include "Object.h"
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

int Object::GetConstraintSize() const {
    return 0;
}

VectorXd Object::GetInnerConstraint(const VectorXd &x) const {
    return VectorXd(0);
}

void Object::GetInnerConstraintGradient(const VectorXd &x, COO &coo, int x_offset, int y_offset) const {

}

Object::~Object() {
    for (auto& ext_force : _external_forces) {
        delete ext_force;
    }
}

Object::Object(const Object &rhs) {
    _x = rhs._x;
    _v = rhs._v;
    for (const auto& ext_force : rhs._external_forces) {
        _external_forces.push_back(ext_force->Clone());
    }
}

ShapedObject::ShapedObject(const Shape &shape)
    : _shape(shape.Clone()) {}

ShapedObject::~ShapedObject() noexcept {
    delete _shape;
}

ShapedObject::ShapedObject(const ShapedObject &rhs) : _shape(rhs._shape->Clone()){}

void ShapedObject::GetShape(Eigen::MatrixXd &vertices, Eigen::MatrixXi &topo) const {
    _shape->GetSurface(*this, vertices, topo);
}

SampledObject::SampledObject(const Eigen::VectorXd &mass) : _mass(mass) {}

void SampledObject::GetMass(COO &coo, int x_offset, int y_offset) const {
    int num_points = _mass.size();
    for (int i = 0, j = 0; i < num_points; i++, j += 3) {
        coo.push_back(Tripletd(x_offset + j, y_offset + j, _mass(i)));
        coo.push_back(Tripletd(x_offset + j + 1, y_offset + j + 1, _mass(i)));
        coo.push_back(Tripletd(x_offset + j + 2, y_offset + j + 2, _mass(i)));
    }
}

#include "Curve/InextensibleCurve.h"
#include "ReducedObject/ReducedBezierCurve.h"
#include "Curve/ExtensibleCurve.h"
#include "Cloth/Cloth.h"

BEGIN_DEFINE_XXX_FACTORY(Object)
    ADD_PRODUCT("inextensible-curve", InextensibleCurve)
    ADD_PRODUCT("extensible-curve", ExtensibleCurve)
    ADD_PRODUCT("reduced-bezier-curve", ReducedBezierCurve)
    ADD_PRODUCT("cloth", Cloth)
END_DEFINE_XXX_FACTORY