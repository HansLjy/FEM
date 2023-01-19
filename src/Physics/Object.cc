//
// Created by hansljy on 10/8/22.
//

#include "Object.h"
#include "JsonUtil.h"
#include "RenderShape/RenderShape.h"
#include "Collision/CollisionShape/CollisionShape.h"
#include "spdlog/spdlog.h"
#include "GeometryUtil.h"

Object::Object(RenderShape* render_shape, CollisionShape* collision_shape) : _render_shape(render_shape), _collision_shape(collision_shape) {}

void Object::Initialize() {
	_collision_shape->Bind(*this);
	_render_shape->Bind(*this);
}

Object::~Object() {
	delete _render_shape;
	delete _collision_shape;
}

ConcreteObject::ConcreteObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x)
	: ConcreteObject(render_shape, collision_shape, x, VectorXd::Zero(x.size())) {}

ConcreteObject::ConcreteObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, const VectorXd& v)
	: Object(render_shape, collision_shape), _x(x), _v(v), _dof(x.size()) {}

int ConcreteObject::GetDOF() const {
    return _dof;
}

void ConcreteObject::GetCoordinate(Ref<VectorXd> x) const {
	x = _x;
}

void ConcreteObject::GetVelocity(Ref<VectorXd> v) const {
	v = _v;
}

void ConcreteObject::SetCoordinate(const Ref<const Eigen::VectorXd> &x) {
    _x = x;
}

void ConcreteObject::SetVelocity(const Ref<const Eigen::VectorXd> &v) {
    _v = v;
}

void ConcreteObject::AddExternalForce(ExternalForce* force) {
	_external_forces.push_back(force);
}

VectorXd ConcreteObject::GetExternalForce() const {
	VectorXd ext_force(GetDOF());
	ext_force.setZero();

	for (const auto& external_force: _external_forces) {
		ext_force -= external_force->EnergyGradient(*this, GetFrameRotation(), GetFrameX());
	}
	return ext_force;
}

Vector3d ConcreteObject::GetFrameX() const {
	return Vector3d::Zero();
}

Matrix3d ConcreteObject::GetFrameRotation() const {
	return Matrix3d::Identity();
}

ConcreteObject::~ConcreteObject() {
	for (const auto& external_force : _external_forces) {
		delete external_force;
	}
}

// Sampled Object

SampledObject::SampledObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, const VectorXd& mass, int dimension, const MatrixXi& topo)
	: SampledObject(render_shape, collision_shape, x, VectorXd::Zero(x.size()), mass, dimension, topo) {}

SampledObject::SampledObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, const VectorXd& v, const VectorXd& mass, int dimension, const MatrixXi& topo)
	: ConcreteObject(render_shape, collision_shape, x, v), _mass(mass) {
	switch (dimension) {
		case 3:
			_tet_topo = topo;
			GenerateSurfaceTopo3D(topo, _face_topo, _edge_topo);
			break;
		case 2:
			_face_topo = topo;
			GenerateSurfaceTopo2D(topo, _edge_topo);
			break;
		case 1:
			_edge_topo = topo;
			break;
		default:
			spdlog::error("Unsupported dimension!");
	}
}

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


// Fixed Object

FixedObject::FixedObject(const json& config)
	: FixedObject(
		FixedRenderShapeFactory::GetFixedRenderShape(config["render-object"]["type"], config["render-object"]),
		bool(config["collision-enabled"])
			? FixedCollisionShapeFactory::GetFixedCollisionShape(config["collision-object"]["type"], config["collision-object"])
			: (CollisionShape*)(new NullCollisionShape)
	) {}

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
	ADD_PRODUCT("decomposed-leaf", DecomposedLeaf)
    ADD_PRODUCT("reduced-bezier-curve", ReducedBezierCurve)
    ADD_PRODUCT("reduced-bezier-surface", ReducedBezierSurface)
	ADD_PRODUCT("reduced-leaf", ReducedLeaf)
	ADD_PRODUCT("reduced-treetrunk", ReducedTreeTrunk)
    ADD_PRODUCT("tree-trunk", ReducedTreeTrunk)
	ADD_PRODUCT("fixed-object", FixedObject)
END_DEFINE_XXX_FACTORY
