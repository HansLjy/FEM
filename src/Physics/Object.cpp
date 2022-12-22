//
// Created by hansljy on 10/8/22.
//

#include "Object.h"
#include "RenderShape/RenderShape.h"

Object::Object(RenderShape* render_shape, CollisionShape* collision_shape, const Eigen::VectorXd &x) : _render_shape(render_shape), _collision_shape(collision_shape), _x(x), _v(x.size()), _dof(x.size()) {
    _v.setZero();
}

Object::Object(RenderShape* render_shape, CollisionShape* collision_shape, const Eigen::VectorXd &x, const Eigen::VectorXd &v) : _render_shape(render_shape), _collision_shape(collision_shape), _x(x), _v(v), _dof(x.size()) {}

int Object::GetDOF() const {
    return _dof;
}

const VectorXd &Object::GetCoordinate() const {
    return _x;
}

const VectorXd &Object::GetVelocity() const {
    return _v;
}

void Object::SetCoordinate(const Ref<const Eigen::VectorXd> &x) {
    _x = x;
}

void Object::SetVelocity(const Ref<const Eigen::VectorXd> &v) {
    _v = v;
}

void Object::AddExternalForce(const ExternalForce &force) {
	_external_forces.push_back(force.Clone());
}

VectorXd Object::GetExternalForce() const {
	VectorXd ext_force(GetDOF());
	ext_force.setZero();

	for (const auto& external_force: _external_forces) {
		ext_force += external_force->EnergyGradient(*this, GetFrameRotation(), GetFrameX());
	}
	return ext_force;
}

#include "RenderShape/RenderShape.h"

void Object::GetRenderShape(Eigen::MatrixXd &vertices, Eigen::MatrixXi &topo) const {
    _render_shape->GetSurface(*this, vertices, topo);
}

#include "Collision/CollisionShape/CollisionShape.h"

void Object::ComputeCollisionShape(const Ref<const Eigen::VectorXd> &x) {
    _collision_shape->ComputeCollisionShape(*this, x);
}

const MatrixXd &Object::GetCollisionVertices() const {
    return _collision_shape->GetCollisionVertices();
}

const MatrixXi &Object::GetCollisionEdgeTopo() const {
    return _collision_shape->GetCollisionEdgeTopo();
}

const MatrixXi &Object::GetCollisionFaceTopo() const {
    return _collision_shape->GetCollisionFaceTopo();
}

Vector3d Object::GetFrameX() const {
	return Vector3d::Zero();
}

Matrix3d Object::GetFrameRotation() const {
	return Matrix3d::Identity();
}

Object::~Object() {
	delete _render_shape;
	delete _collision_shape;
	for (const auto& external_force : _external_forces) {
		delete external_force;
	}
}

Object::Object(const Object& rhs) : _render_shape(rhs._render_shape->Clone()), _collision_shape(rhs._collision_shape->Clone()), _x(rhs._x), _v(rhs._v) {
	for (const auto& external_force : rhs._external_forces) {
		_external_forces.push_back(external_force->Clone());
	}
}

// Sampled Object

SampledObject::SampledObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, const VectorXd& mass) 
	: Object(render_shape, collision_shape, x), _mass(mass) {}

SampledObject::SampledObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, const VectorXd& v, const VectorXd& mass)
	: Object(render_shape, collision_shape, x, v), _mass(mass) {}
	

void SampledObject::GetMass(COO &coo, int x_offset, int y_offset) const {
	const int num_points = GetDOF() / 3;
	for (int i = 0, j = 0; i < num_points; i++, j += 3) {
		coo.push_back(Tripletd(x_offset + j, y_offset + j, _mass(i)));
		coo.push_back(Tripletd(x_offset + j + 1, y_offset + j + 1, _mass(i)));
		coo.push_back(Tripletd(x_offset + j + 2, y_offset + j + 2, _mass(i)));
	}
}

double SampledObject::GetTotalMass() const {
	return _mass.sum();
}

double SampledObject::GetMaxVelocity(const Ref<const VectorXd> &v) const {
	const int num_points = GetDOF() / 3;
	double max_velocity = 0;
	for(int i = 0, j = 0; i < num_points; i++, j += 3) {
		max_velocity = std::max(max_velocity, v.segment(j, 3).norm());
	}
	return max_velocity;
}

VectorXd SampledObject::GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const {
	VectorXd inertial_force(_dof);
	const int num_points = _dof / 3;

	for (int i = 0, j = 0; i < num_points; i++, j += 3) {
		Vector3d x_object = rotation * _x.segment<3>(j);
        Vector3d v_object = rotation * _v.segment<3>(j);
        inertial_force.segment<3>(j) = -_mass(i) * rotation.transpose() * (
            alpha.cross(x_object)
            + omega.cross(omega.cross(x_object))
            + 2 * omega.cross(v_object)
            + a
        );
	}
	return inertial_force;
}

Vector3d SampledObject::GetTotalExternalForce() const {
	VectorXd external_force = GetExternalForce();
	int num_points = external_force.size() / 3;
	Vector3d total_external_force = Vector3d::Zero();
	for (int i = 0, j = 0; i < num_points; i++, j += 3) {
		total_external_force += external_force.segment<3>(j);
	}
	return total_external_force;
}

// Decomposed Object

DecomposedObject::~DecomposedObject() {
	for (const auto& child : _children) {
		delete child;
	}
}

Vector3d DecomposedObject::GetFrameX() const {
	return _frame_x;
}

Matrix3d DecomposedObject::GetFrameRotation() const {
	return _frame_rotation;
}

void DecomposedObject::GetMass(COO &coo, int x_offset, int y_offset) const {
	_proxy->GetMass(coo, x_offset, y_offset);
	SparseToCOO(_lumped_mass, coo, x_offset, y_offset);
}

VectorXd DecomposedObject::GetExternalForce() const {
	return _proxy->GetExternalForce() + _interface_force + GetInertialForce(_frame_v, _frame_a, _frame_angular_velocity, _frame_angular_acceleration, _frame_rotation);
}

void DecomposedObject::Aggregate() {
	Object::ComputeCollisionShape();
	_total_mass = GetTotalMass();
	_total_external_force = GetTotalExternalForce();
	// TODO: make this more efficient
	_extra_vertices.clear();
	_extra_edge_topos.clear();
	_extra_face_topos.clear();


	for (auto& child : _children) {
		child->Aggregate();
		_total_mass += child->GetTotalMass();
		_total_external_force += child->GetTotalExternalForce();
		_extra_vertices.push_back(child->GetCollisionVertices());
		_extra_edge_topos.push_back(child->GetCollisionEdgeTopo());
		_extra_face_topos.push_back(child->GetCollisionFaceTopo());
	}
	
	int num_children = _children.size();
	int dof = GetDOF();
	
	/* Lumped Mass */
	if (_lumped_mass.rows() != dof || _lumped_mass.cols() != dof) {
		_lumped_mass.resize(dof, dof);
	}
	
	for (int i = 0; i < num_children; i++) {
		_lumped_mass += _children[i]->_total_mass * _children_projections[i].transpose() * _children_projections[i];
	}

	/* Interface Force */
	if(_interface_force.size() != dof) {
        _interface_force.resize(dof);
    }
    _interface_force.setZero();

    Matrix3d frame_angular_velocity = HatMatrix(_frame_angular_velocity);
    Matrix3d omega = _frame_rotation.transpose() * frame_angular_velocity;
    Matrix3d omega2 = omega * frame_angular_velocity;
    Matrix3d alpha = _frame_rotation.transpose() * HatMatrix(_frame_angular_acceleration);

    for (int i = 0; i < num_children; i++) {
        Vector3d v_rel = _children[i]->_frame_v - _frame_v;
        Vector3d x_rel = _children[i]->_frame_x - _frame_x;
        _interface_force += _children_projections[i].transpose()
                          * _frame_rotation.transpose() * _children[i]->_frame_rotation
                          * _children[i]->_total_external_force;
        _interface_force -= _children_projections[i].transpose()
                          * _children[i]->_total_mass * (_frame_a + 2 * omega * v_rel + alpha * x_rel + omega2 * x_rel);
    }
}

void ReducedObject::GetMass(COO &coo, int x_offset, int y_offset) const {
	COO coo_full;
    _proxy->GetMass(_base * x + _shift, coo_full, 0, 0);
    SparseMatrixXd mass(_proxy->GetDOF(), _proxy->GetDOF());
    VarName.setFromTriplets(coo_full.begin(), coo_full.end());
    SparseMatrixXd mass_reduced = _base.transpose() * mass * _base;
	SparseToCOO(mass_reduced, coo, x_offset, y_offset)
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