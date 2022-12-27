//
// Created by hansljy on 10/8/22.
//

#include "Object.h"
#include "JsonUtil.h"
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

void Object::AddExternalForce(ExternalForce* force) {
	_external_forces.push_back(force);
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

// Reduced Object

void ReducedObject::GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
	COO coo_full;
    _proxy->GetPotentialHessian(_base * x + _shift, coo_full, 0, 0);
    SparseMatrixXd proxy_hessian(_proxy->GetDOF(), _proxy->GetDOF());
    proxy_hessian.setFromTriplets(coo_full.begin(), coo_full.end());
    SparseMatrixXd reduced_hessian = _base.transpose() * proxy_hessian * _base;
	SparseToCOO(reduced_hessian, coo, x_offset, y_offset);
}

void ReducedObject::GetMass(COO &coo, int x_offset, int y_offset) const {
	COO coo_full;
    _proxy->GetMass(coo_full, 0, 0);
    SparseMatrixXd mass(_proxy->GetDOF(), _proxy->GetDOF());
    mass.setFromTriplets(coo_full.begin(), coo_full.end());
    SparseMatrixXd mass_reduced = _base.transpose() * mass * _base;
	SparseToCOO(mass_reduced, coo, x_offset, y_offset);
}


// Decomposed Object

DecomposedObject::DecomposedObject(Object* proxy, const json& config) : _proxy(proxy) {
	if (config["is-root"]) {
        _frame_x = Json2Vec(config["x"]);
        _frame_v = Vector3d::Zero();
        _frame_a = Vector3d::Zero();
        _frame_rotation = Json2Matrix3d(config["rotation"]);
        _frame_angular_velocity = Vector3d::Zero();
        _frame_angular_acceleration = Vector3d::Zero();
    }
	const auto& children_config = config["children"];
	for (const auto& child_config : children_config) {
		AddChild(*DecomposedObjectFactory::GetDecomposedObject(child_config["type"], child_config), child_config["position"]);
	}
}

DecomposedObject::~DecomposedObject() {
	delete _proxy;
	for (const auto& child : _children) {
		delete child;
	}
}

void DecomposedObject::AddExternalForce(ExternalForce* force) {
	_proxy->AddExternalForce(force);
	for (auto& child : _children) {
		child->AddExternalForce(force->Clone());
	}
}

void DecomposedObject::AddChild(DecomposedObject &child, const json &position) {
	_children.push_back(&child);
	_children_rest_rotations.push_back(Matrix3d(AngleAxisd(double(position["angle"]) / 180.0 * EIGEN_PI, Json2Vec(position["axis"]))));
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

void DecomposedObject::ComputeCollisionShape(const Ref<const VectorXd> &x) {
	// TODO:
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

void DecomposedObject::Initialize() {
	VectorXd a(_proxy->GetDOF());
	a.setZero();
	CalculateChildrenFrame(a);
}

#include "Object/Curve.h"
#include "Object/Cloth.h"
#include "Object/DecomposedTree.h"
#include "Object/ReducedBezierCurve.h"
#include "Object/ReducedBezierSurface.h"
#include "Object/ReducedLeaf.h"
#include "Object/ReducedTreeTrunk.h"
#include "Object/TreeTrunk.h"

BEGIN_DEFINE_XXX_FACTORY(Object)
    ADD_PRODUCT("cloth", Cloth)
    ADD_PRODUCT("curve", Curve)
	ADD_PRODUCT("decomposed-treetrunk", DecomposedTreeTrunk)
    ADD_PRODUCT("reduced-bezier-curve", ReducedBezierCurve)
    ADD_PRODUCT("reduced-bezier-surface", ReducedBezierSurface)
	ADD_PRODUCT("reduced-leaf", ReducedLeaf)
	ADD_PRODUCT("reduced-treetrunk", ReducedTreeTrunk)
    ADD_PRODUCT("tree-trunk", ReducedTreeTrunk)
END_DEFINE_XXX_FACTORY

BEGIN_DEFINE_XXX_FACTORY(DecomposedObject)
	ADD_PRODUCT("decomposed-treetrunk", DecomposedTreeTrunk)
END_DEFINE_XXX_FACTORY