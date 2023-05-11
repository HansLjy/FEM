#pragma once

#include "Object.hpp"
#include "Render/RenderShape.hpp"
#include "Collision/CollisionShape/CollisionShape.h"

class DecomposedObject : public Object {
public:
	DecomposedObject(ProxyObject* proxy, const json& config) : _proxy(proxy), _is_root(config["is-root"]) {}

	void Initialize() override;
	bool IsDecomposed() const override {return true;}

	// <- Recursively aggregate subdomain information	
	virtual void Aggregate() = 0;
	// <- Distribute current domain information to its direct children
	virtual void Distribute(const Ref<const VectorXd>& a) = 0;

    void AddExternalForce(const std::string& type, const json& config) override;
	VectorXd GetExternalForce() const override = 0;

	const std::vector<DecomposedObject*>& GetChildren() const {return _abstract_children;}

	~DecomposedObject() override;

protected:
	void AddChild(DecomposedObject* child, const json& position);

	ProxyObject* _proxy;
	bool _is_root;

	int _num_children = 0;
	std::vector<DecomposedObject*> _abstract_children;
};

class RigidDecomposedObject : public DecomposedObject {
public:
	explicit RigidDecomposedObject(ProxyObject* proxy, const json& config);
	void Initialize() override;

	int GetDOF() const override {return _proxy->GetDOF();};
	void GetCoordinate(Ref<VectorXd> x) const override {_proxy->GetCoordinate(x);}
	void GetVelocity(Ref<VectorXd> v) const override {_proxy->GetVelocity(v);}
	void SetCoordinate(const Ref<const VectorXd> &x) override {_proxy->SetCoordinate(x);}
	void SetVelocity(const Ref<const VectorXd> &v) override {_proxy->SetVelocity(v);}

	void GetMass(COO &coo, int x_offset, int y_offset) const override;

	double GetPotential(const Ref<const VectorXd> &x) const override {return _proxy->GetPotential(x);}
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override {return _proxy->GetPotentialGradient(x);}
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override {_proxy->GetPotentialHessian(x, coo, x_offset, y_offset);}

    void AddExternalForce(const std::string& type, const json& config) override;
	VectorXd GetExternalForce() const override;

	/* Frame relevant */
	Vector3d GetFrameX() const override;
	Matrix3d GetFrameRotation() const override;

	/* Domain relevant */
	void Aggregate() override;
	void Distribute(const Ref<const VectorXd> &a) override = 0;
	
	~RigidDecomposedObject() override;

	friend class DecomposedTreeTrunk;
	friend class DecomposedLeaf;

protected:
	void AddChild(DecomposedObject* child, const json& position);

	std::vector<RigidDecomposedObject*> _children;
	std::vector<Matrix3d> _children_rest_rotations;
	std::vector<SparseMatrixXd> _children_projections; // this will be set by derived class during initialization
	
	VectorXd _interface_force;
	SparseMatrixXd _lumped_mass;

    Vector3d _frame_x;
    Vector3d _frame_v;
    Vector3d _frame_a;
    Vector3d _frame_angular_velocity;
    Vector3d _frame_angular_acceleration;
    Matrix3d _frame_rotation;

	double _total_mass;
	Vector3d _total_external_force;
};

class AffineDecomposedObject : public DecomposedObject {
public:
	AffineDecomposedObject(ProxyObject* proxy, const json& config);

	void Initialize() override;
	int GetDOF() const override;
	void GetCoordinate(Ref<VectorXd> x) const override;
	void GetVelocity(Ref<VectorXd> v) const override;
	void SetCoordinate(const Ref<const VectorXd> &x) override;
	void SetVelocity(const Ref<const VectorXd> &v) override;

	void GetMass(COO &coo, int x_offset, int y_offset) const override;

	double GetPotential(const Ref<const VectorXd> &x) const override;
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override;
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override;

    void AddExternalForce(const std::string& type, const json& config) override;
	VectorXd GetExternalForce() const override;

	Vector3d GetFrameX() const override;
	Matrix3d GetFrameRotation() const override;

	void Aggregate() override;
	void Distribute(const Ref<const VectorXd> &a) override;
	void AddChild(DecomposedObject *child, const json &position);

	~AffineDecomposedObject() override;

	enum class CalculateLevel {
		kValue,
		kGradient,
		kHessian
	};

protected:
	virtual void CalculateRigidRotationInfos(const CalculateLevel& level, const Ref<const VectorXd>& x, std::vector<Matrix3d>& rotations, std::vector<MatrixXd>& rotation_gradient, std::vector<MatrixXd>& rotation_hessian) const = 0;

	double _affine_stiffness;

	int _total_dof;
	std::vector<AffineDecomposedObject*> _children;
	std::vector<Matrix3d> _children_rest_A;

	// The following quantities are w.r.t. local coordinate system
	std::vector<Matrix3d> _children_A;				// part of dof
	std::vector<Matrix3d> _children_A_velocity;		// part of dof
	std::vector<Vector3d> _children_b;				// redundant variable, must be synced with dof of proxy
	std::vector<Vector3d> _children_v;				// redundant variable, must be synced with dof of proxy

	std::vector<MatrixXd> _children_projections; 	// transpose of partial b_i / partial q

	VectorXd _interface_force;
	SparseMatrixXd _lumped_mass;

	// The following quantities are w.r.t. global coordinate system
	Vector3d _frame_x;
	Vector3d _frame_v;
	Vector3d _frame_a;
	Matrix3d _frame_affine;	// not necessarily in SO(3)
	Matrix3d _frame_affine_velocity;
	Matrix3d _frame_affine_acceleration;

	// The following quantities are w.r.t. local coordinate system
	double _total_mass;
	Vector3d _total_external_force;
	Vector3d _unnormalized_mass_center;
	Matrix3d _inertial_tensor;					// sum m * x * x^T
	Matrix3d _total_external_force_torque;		// sum f * x^T
};

DECLARE_XXX_FACTORY(AffineDecomposedObject)
DECLARE_XXX_FACTORY(RigidDecomposedObject)
