//
// Created by hansljy on 10/8/22.
//

#include "Object.h"
#include "Shape.h"

void Object::AddExternalForce(const ExternalForce &force) {
    _external_forces.push_back(force.Clone());
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

Object::Object(const Object &rhs)
    : _frame_x(rhs._frame_x), _frame_rotation(rhs._frame_rotation), _extra_force(rhs._extra_force), _x(rhs._x), _v(rhs._v){
    for (const auto& ext_force : rhs._external_forces) {
        _external_forces.push_back(ext_force->Clone());
    }
}

VectorXd Object::GetExternalForce(const Matrix3d &rotation, const Vector3d &position) const {
    VectorXd external_force(_x.size());
    external_force.setZero();
    for (const auto& ext_force : _external_forces) {
        external_force -= ext_force->EnergyGradient(*this, rotation, position);
    }
    return external_force + _extra_force;
}

ShapedObject::~ShapedObject() {
    delete _shape;
}

ShapedObject::ShapedObject(const Shape &shape) : _shape(shape.Clone()) {}

ShapedObject::ShapedObject(const ShapedObject &rhs) : _shape(rhs._shape->Clone()) {}

void ShapedObject::GetRenderShape(Eigen::MatrixXd &vertices, Eigen::MatrixXi &topo) const {
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

double SampledObject::GetTotalMass() const {
    int num_points = _mass.size();
    double total_mass = 0;
    for (int i = 0; i < num_points; i++) {
        total_mass += _mass(i);
    }
    return total_mass;
}

Vector3d SampledObject::GetTotalExternalForce(const Matrix3d &rotation, const Vector3d &position) const {
    VectorXd force = GetExternalForce(rotation, position);
    int num_points = force.size() / 3;
    Vector3d total_force = Vector3d::Zero();
    for (int i = 0, j = 0; i < num_points; i++, j += 3) {
        total_force += force.segment<3>(j);
    }
    return total_force;
}

VectorXd
SampledObject::GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha,
                                const Matrix3d &rotation) const {
    VectorXd inertial_force(_x.size());
    int num_points = _x.size() / 3;
    for (int i = 0, j = 0; i < num_points; i++, j += 3) {
        Vector3d x_object = rotation * _x.segment<3>(j);
        Vector3d v_object = rotation * _v.segment<3>(j);
        inertial_force.segment<3>(j) = - _mass(i) * rotation.transpose() * (
            alpha.cross(x_object)
            + omega.cross(omega.cross(x_object))
            + 2 * omega.cross(v_object)
            + a
        );
    }
    return inertial_force;
}

#include "Curve/InextensibleCurve.h"
#include "ReducedObject/ReducedBezierCurve.h"
#include "Curve/ExtensibleCurve.h"
#include "Cloth/Cloth.h"
#include "ReducedObject/ReducedBezierSurface.h"
#include "Tree/ReducedTreeTrunk.h"
#include "Tree/ReducedLeaf.h"

BEGIN_DEFINE_XXX_FACTORY(Object)
    ADD_PRODUCT("inextensible-curve", InextensibleCurve)
    ADD_PRODUCT("extensible-curve", ExtensibleCurve)
    ADD_PRODUCT("reduced-bezier-curve", ReducedBezierCurve)
    ADD_PRODUCT("cloth", Cloth)
    ADD_PRODUCT("reduced-bezier-surface", ReducedBezierSurface)
    ADD_PRODUCT("tree-trunk", ReducedTreeTrunk)
    ADD_PRODUCT("leaf", ReducedLeaf)
END_DEFINE_XXX_FACTORY