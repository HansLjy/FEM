#include "DecomposedObject.h"
#include "JsonUtil.h"
#include "RenderShape/RenderShape.h"
#include "Collision/CollisionShape/CollisionShape.h"
#include "unsupported/Eigen/KroneckerProduct"

DecomposedObject::DecomposedObject(ProxyObject* proxy, const json& config) : Object(new DecomposedRenderShape, new DecomposedCollisionShape), _proxy(proxy), _is_root(config["is-root"]) {}

void DecomposedObject::Initialize() {
	Object::Initialize();
	_proxy->Initialize();
}

void DecomposedObject::AddExternalForce(ExternalForce *force) {
	_proxy->AddExternalForce(force);
}

void DecomposedObject::AddChild(DecomposedObject &child, const json &position) {
	_num_children++;
	_abstract_children.push_back(&child);
}

DecomposedObject::~DecomposedObject() {
	delete _proxy;
}

RigidDecomposedObject::RigidDecomposedObject(ProxyObject* proxy, const json& config)
	: DecomposedObject(proxy, config) {
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

void RigidDecomposedObject::AddChild(DecomposedObject &child, const json &position) {
	DecomposedObject::AddChild(child, position);
	_children.push_back(dynamic_cast<RigidDecomposedObject*>(&child));
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
	return _proxy->GetExternalForceWithFrame(_frame_rotation, _frame_x) + _interface_force + _proxy->GetInertialForce(_frame_v, _frame_a, _frame_angular_velocity, _frame_angular_acceleration, _frame_rotation);
}

void RigidDecomposedObject::Aggregate() {
	_total_mass = _proxy->GetTotalMass();
	_total_external_force = _proxy->GetTotalExternalForce(_frame_rotation, _frame_x);

	for (auto& child : _children) {
		child->Aggregate();
		_total_mass += child->_total_mass;
		_total_external_force += child->_total_external_force;
	}

	/* Lumped Mass */
	_lumped_mass.setZero();
	for (int i = 0; i < _num_children; i++) {
		_lumped_mass += _children[i]->_total_mass * _children_projections[i].transpose() * _children_projections[i];
	}

	/* Interface Force */
    _interface_force.setZero();
    Matrix3d frame_angular_velocity = HatMatrix(_frame_angular_velocity);
    Matrix3d omega = _frame_rotation.transpose() * frame_angular_velocity;
    Matrix3d omega2 = omega * frame_angular_velocity;
    Matrix3d alpha = _frame_rotation.transpose() * HatMatrix(_frame_angular_acceleration);

    for (int i = 0; i < _num_children; i++) {
        Vector3d v_rel = _children[i]->_frame_v - _frame_v;
        Vector3d x_rel = _children[i]->_frame_x - _frame_x;
        _interface_force += _children_projections[i].transpose()
                          * _frame_rotation.transpose() * _children[i]->_frame_rotation
                          * _children[i]->_total_external_force;
        _interface_force -= _children_projections[i].transpose()
                          * _children[i]->_total_mass * (_frame_a + 2 * omega * v_rel + alpha * x_rel + omega2 * x_rel);
    }
}

void RigidDecomposedObject::Initialize() {
	DecomposedObject::Initialize();
	VectorXd a = VectorXd::Zero(_proxy->GetDOF());
	CalculateChildrenFrame(a);

	for (const auto child : _children) {
		child->Initialize();
	}
}

AffineDecomposedObject::AffineDecomposedObject(ProxyObject* proxy, const json& config)
	: DecomposedObject(proxy, config), _affine_stiffness(config["affine-stiffness"]) {
	if (config["is-root"]) {
		_frame_x = Json2Vec(config["x"]);
		_frame_v = Vector3d::Zero();
		_frame_a = Vector3d::Zero();
		_frame_affine = Json2Matrix3d(config["affine"]);
		_frame_affine_velocity = Matrix3d::Zero();
		_frame_affine_acceleration = Matrix3d::Zero();
	}

	const auto& children_config = config["children"];
	for (const auto& child_config : children_config) {
		AddChild(*AffineDecomposedObjectFactory::GetAffineDecomposedObject(child_config["type"], child_config), child_config["position"]);
	}
	_total_dof = _proxy->GetDOF() + 9 * _num_children;
	_interface_force.resize(_total_dof);
	_lumped_mass.resize(_total_dof, _total_dof);
}

void AffineDecomposedObject::Initialize() {
	DecomposedObject::Initialize();

	VectorXd a = VectorXd::Zero(_total_dof);
	CalculateChildrenFrame(a);

	for (auto& child : _children) {
		child->Initialize();
	}
}

int AffineDecomposedObject::GetDOF() const {
	return _total_dof;
}

void AffineDecomposedObject::GetCoordinate(Ref<VectorXd> x) const {
	_proxy->GetCoordinate(x.head(_proxy->GetDOF()));
	int current_offset = _proxy->GetDOF();
	for (const auto& child_affine : _children_A) {
		for (int i = 0, ii = 0; i < 3; i++, ii += 3) {
			x.segment<3>(current_offset + ii) = child_affine.col(i);
		}
		current_offset += 9;
	}
}

void AffineDecomposedObject::GetVelocity(Ref<VectorXd> v) const {
	_proxy->GetVelocity(v.head(_proxy->GetDOF()));
	int current_offset = _proxy->GetDOF();
	for (const auto& child_affine_velocity : _children_A_velocity) {
		for (int i = 0, ii = 0; i < 3; i++, ii += 3) {
			v.segment<3>(current_offset + ii) = child_affine_velocity.col(i);
		}
		current_offset += 9;
	}
}

void AffineDecomposedObject::SetCoordinate(const Ref<const VectorXd> &x) {
	VectorXd x_proxy = x.head(_proxy->GetDOF());
	_proxy->SetCoordinate(x_proxy);
	int current_offset = _proxy->GetDOF();
	for (auto& child_A : _children_A) {
		for (int i = 0, ii = 0; i < 3; i++, ii += 3) {
			child_A.col(i) = x.segment<3>(current_offset + ii);
		}
		current_offset += 9;
	}

	for (int i = 0; i < _num_children; i++) {
		_children_b[i] = _children_projections[i] * x_proxy;
	}
}

void AffineDecomposedObject::SetVelocity(const Ref<const VectorXd> &v) {
	VectorXd v_proxy = v.head(_proxy->GetDOF());
	_proxy->SetVelocity(v_proxy);
	int current_offset = _proxy->GetDOF();
	for (auto& child_A_velocity : _children_A_velocity) {
		for (int i = 0, ii = 0; i < 3; i++, ii += 3) {
			child_A_velocity.col(i) = v.segment<3>(current_offset + ii);
		}
		current_offset += 9;
	}
	
	for (int i = 0; i < _num_children; i++) {
		_children_v[i] = _children_projections[i] * v_proxy;
	}
}

double AffineDecomposedObject::GetMaxVelocity(const Ref<const VectorXd> &v) const {
	// TODO: I think this function is not supposed to be called
}

void AffineDecomposedObject::GetMass(COO &coo, int x_offset, int y_offset) const {
	_proxy->GetMass(coo, x_offset, y_offset);
	SparseToCOO(_lumped_mass, coo, x_offset, y_offset);
}

double AffineDecomposedObject::GetPotential(const Ref<const VectorXd> &x) const {
	std::vector<Matrix3d> children_rotations;
	std::vector<MatrixXd> null1, null2;
	CalculateRigidRotationInfos(CalculateLevel::kValue, x.head(_proxy->GetDOF()), children_rotations, null1, null2); 

	double potential = _proxy->GetPotential(x.head(_proxy->GetDOF()));
	for (int i = 0, cur_offset = _proxy->GetDOF(); i < _num_children; i++, cur_offset += 9) {
		for (int j = 0, j3 = 0; j < 3; j++, j3 += 3) {
			potential += _affine_stiffness * (children_rotations[i].col(j) - x.segment<3>(cur_offset + j3)).squaredNorm();
		}
	}
	return potential;
}

VectorXd AffineDecomposedObject::GetPotentialGradient(const Ref<const VectorXd> &x) const {
	const int proxy_dof = _proxy->GetDOF();
	std::vector<Matrix3d> children_rotations;
	std::vector<MatrixXd> children_rotations_gradient, null;
	CalculateRigidRotationInfos(CalculateLevel::kGradient, x.head(proxy_dof), children_rotations, children_rotations_gradient, null);

	VectorXd gradient(_total_dof);
	gradient.head(_proxy->GetDOF()) = _proxy->GetPotentialGradient(x.head(proxy_dof));

	for (int i = 0, cur_offset = proxy_dof; i < _num_children; i++, cur_offset += 9) {
		Vector9d tmp; // 2 * vec(Ai - Ri)
		for (int j = 0, jj = 0; j < 3; j++, jj += 3) {
			tmp.segment<3>(jj) = 2 * (x.segment<3>(cur_offset + jj) - children_rotations[i].col(j));
		}
		gradient.segment<9>(cur_offset) = _affine_stiffness * tmp;
		gradient.head(proxy_dof) -= _affine_stiffness * children_rotations_gradient[i] * tmp;
	}
	return gradient;
}

void AffineDecomposedObject::GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
	const int proxy_dof = _proxy->GetDOF();
	std::vector<Matrix3d> children_rotations;
	std::vector<MatrixXd> children_rotations_gradient, children_rotations_hessian;
	CalculateRigidRotationInfos(CalculateLevel::kHessian, x.head(proxy_dof), children_rotations, children_rotations_gradient, children_rotations_hessian);

	// if (_num_children != 0) {
	// 	std::cerr << "Rotation:\n" << children_rotations[0] << std::endl;
	// }

	for (int i = proxy_dof; i < _total_dof; i++)	{
		coo.push_back(Tripletd(x_offset + i, y_offset + i, 2 * _affine_stiffness));
	}

	MatrixXd top_left_hession = MatrixXd::Zero(proxy_dof, proxy_dof);

	for (int i = 0, cur_offset = proxy_dof; i < _num_children; i++, cur_offset += 9) {
		const auto& child_rotation = children_rotations[i];
		const auto& child_rotation_gradient = children_rotations_gradient[i];
		const auto& child_rotation_hessian = children_rotations_hessian[i];
		for (int j = 0; j < child_rotation_gradient.rows(); j++) {
			for (int k = 0; k < child_rotation_gradient.cols(); k++) {
				coo.push_back(Tripletd(x_offset + j + cur_offset, y_offset + k, -2 * child_rotation_gradient(k, j) * _affine_stiffness));
				coo.push_back(Tripletd(y_offset + k, x_offset + j + cur_offset, -2 * child_rotation_gradient(k, j) * _affine_stiffness));
			}
		}
		top_left_hession += 2 * child_rotation_gradient * child_rotation_gradient.transpose() * _affine_stiffness;
		Vector9d pfpR = -2 * x.segment<9>(cur_offset);
		for (int j = 0, j3 = 0; j < 3; j++, j3 += 3) {
			pfpR.segment<3>(j3) += 2 * child_rotation.col(j);
		}
		for (int j = 0, j_proxy = 0; j < 9; j++, j_proxy += proxy_dof) {
			top_left_hession += pfpR(j) * child_rotation_hessian.middleCols(j_proxy, proxy_dof) * _affine_stiffness;
		}
	}
	for (int i = 0; i < proxy_dof; i++) {
		for (int j = 0; j < proxy_dof; j++) {
			coo.push_back(Tripletd(x_offset + i, x_offset + j, top_left_hession(i, j)));
		}
	}

	_proxy->GetPotentialHessian(x.head(proxy_dof), coo, x_offset, y_offset);
}

void AffineDecomposedObject::AddExternalForce(ExternalForce *force) {
	DecomposedObject::AddExternalForce(force);
	for (auto child : _children) {
		child->AddExternalForce(force->Clone());
	}
}

VectorXd AffineDecomposedObject::GetExternalForce() const {
	VectorXd ext_force = _interface_force;
	ext_force.head(_proxy->GetDOF()) += _proxy->GetExternalForceWithFrame(_frame_affine, _frame_x) + _proxy->GetInertialForce(_frame_v, _frame_a, _frame_affine, _frame_affine_velocity, _frame_affine_acceleration);
	return ext_force;
}

Vector3d AffineDecomposedObject::GetFrameX() const {
	return _frame_x;
}

Matrix3d AffineDecomposedObject::GetFrameRotation() const {
	return _frame_affine;
}

void AffineDecomposedObject::Aggregate() {
	_total_mass = _proxy->GetTotalMass();
	_total_external_force = _proxy->GetTotalExternalForce(_frame_affine, _frame_x);
	_unnormalized_mass_center = _proxy->GetUnnormalizedMassCenter();
	_inertial_tensor = _proxy->GetInertialTensor();
	_total_external_force_torque = _proxy->GetTotalExternalForceTorque(_frame_affine, _frame_x);

	for (int i = 0; i < _num_children; i++) {
		auto& child = _children[i];
		const auto& A = _children_A[i];
		const auto& b = _children_b[i];
		child->Aggregate();
		_total_mass += child->_total_mass;
		_total_external_force += A * child->_total_external_force;
		_unnormalized_mass_center += A * child->_unnormalized_mass_center + child->_total_mass * b;
		_inertial_tensor += A * child->_inertial_tensor * A.transpose() + child->_total_mass * b * b.transpose() + A * child->_unnormalized_mass_center * b.transpose() + b * child->_unnormalized_mass_center.transpose() * A.transpose();
		_total_external_force_torque += A * child->_total_external_force * b.transpose() + A * child->_total_external_force_torque * A.transpose();
	}

	COO coo;
	const int proxy_dof = _proxy->GetDOF();
	MatrixXd proxy_lumped_mass = MatrixXd::Zero(proxy_dof, proxy_dof);
	for (int i = 0, cur_offset = proxy_dof; i < _num_children; i++, cur_offset += 9) {
		proxy_lumped_mass += _children[i]->_total_mass * _children_projections[i].transpose() * _children_projections[i];

		MatrixXd proxy_affine_projection = Eigen::KroneckerProduct<Vector3d, Matrix3d>(_children[i]->_unnormalized_mass_center, Matrix3d::Identity()) * _children_projections[i];

		DenseToCOO(
			proxy_affine_projection,
			coo, cur_offset, 0
		);

		DenseToCOO(
			proxy_affine_projection.transpose(),
			coo, 0, cur_offset
		);

		const auto& child_inertial_tensor = _children[i]->_inertial_tensor;

		for (int j = 0, j3 = cur_offset; j < 3; j++, j3 += 3) {
			for (int k = 0, k3 = cur_offset; k < 3; k++, k3 += 3) {
				for (int l = 0; l < 3; l++) {
					coo.push_back(Tripletd(j3 + l, k3 + l, child_inertial_tensor(j, k)));
				}
			}
		}
	}

	DenseToCOO(proxy_lumped_mass, coo, 0, 0);

	_lumped_mass.setZero();
	_lumped_mass.setFromTriplets(coo.begin(), coo.end());

	_interface_force.setZero();
	for (int i = 0, cur_offset = proxy_dof; i < _num_children; i++, cur_offset += 9) {
		_interface_force.head(proxy_dof) += _children_projections[i].transpose() * _children[i]->_total_external_force;
		Matrix3d tmp = _children_A[i] * _children[i]->_total_external_force_torque;
		for (int j = 0, j3 = cur_offset; j < 3; j++, j3 += 3) {
			_interface_force.segment<3>(j3) = tmp.col(j);
		}
	}
}

void AffineDecomposedObject::AddChild(DecomposedObject &child, const json &position) {
	DecomposedObject::AddChild(child, position);
	_children.push_back(dynamic_cast<AffineDecomposedObject*>(&child));
	Matrix3d rest_A = Matrix3d(AngleAxisd(double(position["angle"]) / 180.0 * EIGEN_PI, Json2Vec(position["axis"])));
	_children_A.push_back(rest_A);
	_children_rest_A.push_back(rest_A);
	_children_A_velocity.push_back(Matrix3d::Zero());
}

void AffineDecomposedObject::CalculateChildrenFrame(const Ref<const VectorXd> &a) {
	for (int i = 0, cur_offset = _proxy->GetDOF(); i < _num_children; i++, cur_offset += 9) {
		_children[i]->_frame_x = _frame_x + _children_b[i];
		_children[i]->_frame_v = _frame_affine_velocity * _children_b[i] + _frame_affine * _children_v[i] + _frame_v;
		_children[i]->_frame_a = _frame_affine_acceleration * _children_b[i]
							   + 2 * _frame_affine_velocity * _children_v[i]
							   + _frame_affine * _children_projections[i] * a.head(_proxy->GetDOF())
							   + _frame_a;
		
		_children[i]->_frame_affine = _frame_affine * _children_A[i];
		_children[i]->_frame_affine_velocity = _frame_affine_velocity * _children_A[i] + _frame_affine * _children_A_velocity[i];

		Matrix3d children_affine_acceleration;
		for (int i = 0, i3 = 0; i < 3; i++, i3 += 3) {
			children_affine_acceleration.col(i) = a.segment<3>(cur_offset + i3);
		}
		_children[i]->_frame_affine_acceleration = _frame_affine_acceleration * _children_A[i]
												 + 2 * _frame_affine_velocity * _children_A_velocity[i]
												 + _frame_affine * children_affine_acceleration;
	}
}

AffineDecomposedObject::~AffineDecomposedObject() {
	for (auto child : _children) {
		delete child;
	}
}

#include "Object/DecomposedTree.h"

BEGIN_DEFINE_XXX_FACTORY(RigidDecomposedObject)
	ADD_PRODUCT("decomposed-treetrunk", DecomposedTreeTrunk)
	ADD_PRODUCT("decomposed-leaf", DecomposedLeaf)
END_DEFINE_XXX_FACTORY

#include "Object/AffineDecomposedTree.h"

BEGIN_DEFINE_XXX_FACTORY(AffineDecomposedObject)
	ADD_PRODUCT("affine-decomposed-treetrunk", AffineDecomposedTreeTrunk)
END_DEFINE_XXX_FACTORY