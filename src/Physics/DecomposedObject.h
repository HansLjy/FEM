#pragma once

#include "Object.h"

class DecomposedRenderShape;

class DecomposedObject : public Object {
public:
	DecomposedObject(ProxyObject* proxy, const json& config);

	void Initialize() override;

	int GetDOF() const override = 0;
	void GetCoordinate(Ref<VectorXd> x) const override = 0;
	void GetVelocity(Ref<VectorXd> v) const override = 0;
	void SetCoordinate(const Ref<const VectorXd> &x) override = 0;
	void SetVelocity(const Ref<const VectorXd> &v) override = 0;

	double GetMaxVelocity(const Ref<const VectorXd> &v) const override = 0;
	
	void GetMass(COO &coo, int x_offset, int y_offset) const override = 0;

	double GetPotential(const Ref<const VectorXd> &x) const override = 0;
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override = 0;
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override = 0;

	void AddExternalForce(ExternalForce *force) override;
	VectorXd GetExternalForce() const override = 0;

	Vector3d GetFrameX() const override = 0;
	Matrix3d GetFrameRotation() const override = 0;

	bool IsDecomposed() const override {return true;}
	
	/**
	 * @brief Aggregate infomation from bottom to top
	 * @note This is a recursive function
	 */
	virtual void Aggregate() = 0;

	const std::vector<DecomposedObject*>& GetChildren() {
		return _abstract_children;
	}

	void AddChild(DecomposedObject& child, const json& position);

	/**
	 * @warning This is non-recursive
	 */
	virtual void CalculateChildrenFrame(const Ref<const VectorXd>& a) = 0;

	~DecomposedObject() override;

	friend class DecomposedRenderShape;

protected:
	ProxyObject* _proxy;
	bool _is_root;

	int _num_children = 0;
	std::vector<DecomposedObject*> _abstract_children;
};

class RigidDecomposedObject : public DecomposedObject {
public:
	explicit RigidDecomposedObject(ProxyObject* proxy, const json& config);

	int GetDOF() const override {return _proxy->GetDOF();};
	void GetCoordinate(Ref<VectorXd> x) const override {_proxy->GetCoordinate(x);}
	void GetVelocity(Ref<VectorXd> v) const override {_proxy->GetVelocity(v);}
	void SetCoordinate(const Ref<const VectorXd> &x) override {_proxy->SetCoordinate(x);}
	void SetVelocity(const Ref<const VectorXd> &v) override {_proxy->SetVelocity(v);}

	double GetMaxVelocity(const Ref<const VectorXd> &v) const override {return _proxy->GetMaxVelocity(v);}

	void GetMass(COO &coo, int x_offset, int y_offset) const override;

	double GetPotential(const Ref<const VectorXd> &x) const override {return _proxy->GetPotential(x);}
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override {return _proxy->GetPotentialGradient(x);}
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override {_proxy->GetPotentialHessian(x, coo, x_offset, y_offset);}

	void AddExternalForce(ExternalForce *force) override;
	VectorXd GetExternalForce() const override;

	/* Frame relevant */
	Vector3d GetFrameX() const override;
	Matrix3d GetFrameRotation() const override;

	/* Domain relevant */
	void CalculateChildrenFrame(const Ref<const VectorXd>& a) override = 0;
	
	void Aggregate() override;
	void AddChild(DecomposedObject& child, const json& position);

	bool IsDecomposed() const override {return true;}
	void Initialize() override;
	
	~RigidDecomposedObject() override;

	friend class System;
	friend class DecomposedTreeTrunk;

protected:
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

	double GetMaxVelocity(const Ref<const VectorXd> &v) const override;
	void GetMass(COO &coo, int x_offset, int y_offset) const override;

	double GetPotential(const Ref<const VectorXd> &x) const override;
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override;
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override;

	void AddExternalForce(ExternalForce *force) override;
	VectorXd GetExternalForce() const override;

	Vector3d GetFrameX() const override;
	Matrix3d GetFrameRotation() const override;

	void Aggregate() override;
	void CalculateChildrenFrame(const Ref<const VectorXd> &a) override;
	void AddChild(DecomposedObject &child, const json &position);

	~AffineDecomposedObject() override;

	enum class CalculateLevel {
		kValue,
		kGradient,
		kHessian
	};

protected:
	virtual void CalculateRigidRotationInfos(const CalculateLevel& level, const Ref<const VectorXd>& x, std::vector<Matrix3d>& rotations, std::vector<MatrixXd>& rotation_gradient, std::vector<MatrixXd>& rotation_hessian) const = 0;

	int _total_dof;
	std::vector<AffineDecomposedObject*> _children;

	// The following quantities are w.r.t. local coordinate system
	std::vector<Matrix3d> _children_A;				// part of dof
	std::vector<Matrix3d> _children_A_velocity;		// part of dof
	std::vector<Vector3d> _children_b;				// redundant variable, must be synced with dof of proxy
	std::vector<Vector3d> _children_v;				// redundant variable, must be synced with dof of proxy

	std::vector<MatrixXd> _children_projections; // partial b_i / partial q

	VectorXd _interface_force;		// only for the dof of proxy
	SparseMatrixXd _lumped_mass;	// for whole dof

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
	Matrix3d _inertial_tensor;
};

DECLARE_XXX_FACTORY(AffineDecomposedObject)
DECLARE_XXX_FACTORY(RigidDecomposedObject)
