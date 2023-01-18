#pragma once

#include "Object.h"

class DecomposedRenderShape;

class RigidDecomposedObject : public Object {
public:
	explicit RigidDecomposedObject(Object* proxy, const json& config);

	int GetDOF() const override {return _proxy->GetDOF();};
	void GetCoordinate(Ref<VectorXd> x) const override {_proxy->GetCoordinate(x);}
	void GetVelocity(Ref<VectorXd> v) const override {_proxy->GetVelocity(v);}
	void SetCoordinate(const Ref<const VectorXd> &x) override {_proxy->SetCoordinate(x);}
	void SetVelocity(const Ref<const VectorXd> &v) override {_proxy->SetVelocity(v);}

	double GetMaxVelocity(const Ref<const VectorXd> &v) const override {return _proxy->GetMaxVelocity(v);}

	void GetMass(COO &coo, int x_offset, int y_offset) const override;
	double GetTotalMass() const override {return _proxy->GetTotalMass();}

	double GetPotential(const Ref<const VectorXd> &x) const override {return _proxy->GetPotential(x);}
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override {return _proxy->GetPotentialGradient(x);}
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override {_proxy->GetPotentialHessian(x, coo, x_offset, y_offset);}

	void AddExternalForce(ExternalForce *force) override;
	VectorXd GetExternalForce() const override;
	Vector3d GetTotalExternalForce() const override {return _proxy->GetTotalExternalForce();}

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override {return _proxy->GetInertialForce(v, a, omega, alpha, rotation);}

	/* Frame relevant */
	Vector3d GetFrameX() const override;
	Matrix3d GetFrameRotation() const override;

	/* Domain relevant */
	/**
	 * @brief Calculate frame of the children
	 * @note This is a non-recursive function
	 */
	virtual void CalculateChildrenFrame(const Ref<const VectorXd>& a) = 0;
	/**
	 * @brief Aggregate infomation from bottom to top
	 * @note This is a recursive function
	 */
	void Aggregate();
	void AddChild(RigidDecomposedObject& child, const json& position);

	bool IsDecomposed() const override {return true;}
	void Initialize() override;
	
	~RigidDecomposedObject() override;
	RigidDecomposedObject(const RigidDecomposedObject& rhs);

	friend class System;
	friend class DecomposedTreeTrunk;
	friend class DecomposedRenderShape;

protected:
	Object* _proxy;

	bool _is_root;

	std::vector<RigidDecomposedObject*> _children;
	std::vector<Matrix3d> _children_rest_rotations;
	std::vector<SparseMatrixXd> _children_projections; // this will be set by derived class during initialization
	
	VectorXd _interface_force;
	Vector3d _extra_total_force;
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

DECLARE_XXX_FACTORY(RigidDecomposedObject)
