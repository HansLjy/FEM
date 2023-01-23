#include "DecomposedObject.h"
#include "JsonUtil.h"
#include "RenderShape/RenderShape.h"
#include "Collision/CollisionShape/CollisionShape.h"

DecomposedObject::DecomposedObject(RenderShape* render_shape, CollisionShape* collision_shape, Object* proxy, bool is_root) : Object(render_shape, collision_shape), _proxy(proxy), _is_root(is_root) {}

void DecomposedObject::Initialize() {
	Object::Initialize();
	_proxy->Initialize();
}

void DecomposedObject::AddExternalForce(ExternalForce *force) {
	_proxy->AddExternalForce(force);
}

VectorXd DecomposedObject::GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const {
	return _proxy->GetInertialForce(v, a, omega, alpha, rotation);
}

DecomposedObject::~DecomposedObject() {
	delete _proxy;
}

RigidDecomposedObject::RigidDecomposedObject(Object* proxy, const json& config)
	: DecomposedObject(new DecomposedRenderShape, new NullCollisionShape, proxy, config["is-root"]) {
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
		AddChild(*RigidDecomposedObjectFactory::GetRigidDecomposedObject(child_config["type"], child_config), child_config["position"]);
	}

	_interface_force.resize(_proxy->GetDOF());
	_lumped_mass.resize(_proxy->GetDOF(), _proxy->GetDOF());
}

RigidDecomposedObject::~RigidDecomposedObject() {
	for (const auto& child : _children) {
		delete child;
	}
}

void RigidDecomposedObject::AddExternalForce(ExternalForce* force) {
	DecomposedObject::AddExternalForce(force);
	for (auto& child : _children) {
		child->AddExternalForce(force->Clone());
	}
}

void RigidDecomposedObject::AddChild(RigidDecomposedObject &child, const json &position) {
	_children.push_back(&child);
	_children_rest_rotations.push_back(Matrix3d(AngleAxisd(double(position["angle"]) / 180.0 * EIGEN_PI, Json2Vec(position["axis"]))));
}

Vector3d RigidDecomposedObject::GetFrameX() const {
	return _frame_x;
}

Matrix3d RigidDecomposedObject::GetFrameRotation() const {
	return _frame_rotation;
}

void RigidDecomposedObject::GetMass(COO &coo, int x_offset, int y_offset) const {
	_proxy->GetMass(coo, x_offset, y_offset);
	SparseToCOO(_lumped_mass, coo, x_offset, y_offset);
}

VectorXd RigidDecomposedObject::GetExternalForce() const {
	return _proxy->GetExternalForce() + _interface_force + GetInertialForce(_frame_v, _frame_a, _frame_angular_velocity, _frame_angular_acceleration, _frame_rotation);
}

void RigidDecomposedObject::Aggregate() {
	_total_mass = GetTotalMass();
	_total_external_force = GetTotalExternalForce();

	for (auto& child : _children) {
		child->Aggregate();
		_total_mass += child->GetTotalMass();
		_total_external_force += child->GetTotalExternalForce();
	}
	
	int num_children = _children.size();
	
	/* Lumped Mass */
	_lumped_mass.setZero();
	for (int i = 0; i < num_children; i++) {
		_lumped_mass += _children[i]->_total_mass * _children_projections[i].transpose() * _children_projections[i];
	}

	/* Interface Force */
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

std::vector<DecomposedObject *> RigidDecomposedObject::GetChildren() {
	std::vector<DecomposedObject *> result;
	for (auto child : _children) {
		result.push_back(child);
	}
	return result;
}

void RigidDecomposedObject::Initialize() {
	DecomposedObject::Initialize();
	for (const auto child : _children) {
		child->Initialize();
	}

	if (_is_root) {
		VectorXd a = VectorXd::Zero(_proxy->GetDOF());
		CalculateChildrenFrame(a);
	}
}

#include "Object/DecomposedTree.h"

BEGIN_DEFINE_XXX_FACTORY(RigidDecomposedObject)
	ADD_PRODUCT("decomposed-treetrunk", DecomposedTreeTrunk)
	ADD_PRODUCT("decomposed-leaf", DecomposedLeaf)
END_DEFINE_XXX_FACTORY