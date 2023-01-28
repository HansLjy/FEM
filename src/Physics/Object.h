//
// Created by hansljy on 10/7/22.
//

#ifndef FEM_OBJECT_H
#define FEM_OBJECT_H

#include "EigenAll.h"
#include "Pattern.h"
#include "ExternalForce/ExternalForce.h"

class RenderShape;
class CollisionShape;
class RigidDecomposedObject;

enum class CollisionAssemblerType {
	kNull,
	kIndex,
};

class Object {
public:
    Object(RenderShape* render_shape, CollisionShape* collision_shape);

	virtual void Initialize();

	virtual int GetDOF() const = 0;
	virtual void GetCoordinate(Ref<VectorXd> x) const = 0;
	virtual void GetVelocity(Ref<VectorXd> v) const = 0;
	virtual void SetCoordinate(const Ref<const VectorXd>& x) = 0;
	virtual void SetVelocity(const Ref<const VectorXd>& v) = 0;

	virtual double GetMaxVelocity(const Ref<const VectorXd>& v) const = 0;

	/* Physics relevant */
    virtual void GetMass(COO& coo, int x_offset, int y_offset) const = 0;
    virtual double GetTotalMass() const = 0;

	virtual Vector3d GetUnnormalizedMassCenter() const = 0;
	virtual Matrix3d GetInertialTensor() const = 0;

    virtual double GetPotential(const Ref<const VectorXd>& x) const = 0;
    virtual VectorXd GetPotentialGradient(const Ref<const VectorXd>& x) const = 0;
    virtual void GetPotentialHessian(const Ref<const VectorXd>& x, COO& coo, int x_offset, int y_offset) const = 0;

    virtual void AddExternalForce(ExternalForce* force) = 0;
    virtual VectorXd GetExternalForce() const = 0;
    virtual Vector3d GetTotalExternalForce() const = 0;

	virtual VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const = 0;
	// The inertial force in **local coordinate**
	virtual VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d& affine_velocity, const Matrix3d& affine_acceleration) const = 0;

	/* Frame relevant */
	virtual Vector3d GetFrameX() const = 0;
	virtual Matrix3d GetFrameRotation() const = 0;

	virtual bool IsDecomposed() const = 0;

	virtual ~Object();
	Object(const Object& rhs) = delete;
	Object& operator=(const Object& rhs) = delete;

	RenderShape* _render_shape;
    CollisionShape* _collision_shape;
};

class ConcreteObject : public Object {
public:
    ConcreteObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x);
    ConcreteObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, const VectorXd& v);

	int GetDOF() const override;
	void GetCoordinate(Ref<VectorXd> x) const override;
	void GetVelocity(Ref<VectorXd> v) const override;
	void SetCoordinate(const Ref<const VectorXd> &x) override;
	void SetVelocity(const Ref<const VectorXd> &v) override;

	double GetMaxVelocity(const Ref<const VectorXd> &v) const override = 0;

	Vector3d GetUnnormalizedMassCenter() const override = 0;
	Matrix3d GetInertialTensor() const override = 0;

    /* Physics relevant */
	void GetMass(COO &coo, int x_offset, int y_offset) const override = 0;
	double GetTotalMass() const override = 0;

	double GetPotential(const Ref<const VectorXd> &x) const override = 0;
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override = 0;
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override = 0;

	void AddExternalForce(ExternalForce *force) override;
	VectorXd GetExternalForce() const override;
	Vector3d GetTotalExternalForce() const override = 0;

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override = 0;
	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d &affine_velocity, const Matrix3d &affine_acceleration) const override = 0;

	/* Frame relevant */
	Vector3d GetFrameX() const override;
	Matrix3d GetFrameRotation() const override;

	bool IsDecomposed() const override {return false;}

	~ConcreteObject() override;

	friend class RigidDecomposedObject;

protected:
    std::vector<ExternalForce*> _external_forces;

    VectorXd _x;
    VectorXd _v;
	int _dof;
};

class SampledRenderShape;
class SampledCollisionShape;
class SampledObjectGravity;

class SampledObject : public ConcreteObject {
public:
	SampledObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, const VectorXd& mass, int dimension, const MatrixXi& topo);
	SampledObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, const VectorXd& v, const VectorXd& mass, int dimension, const MatrixXi& topo);
	
	double GetMaxVelocity(const Ref<const VectorXd> &v) const override;

	Vector3d GetUnnormalizedMassCenter() const override;
	Matrix3d GetInertialTensor() const override;

	void GetMass(COO &coo, int x_offset, int y_offset) const override;
	double GetTotalMass() const override;

	Vector3d GetTotalExternalForce() const override;

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override;
	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d &affine_velocity, const Matrix3d &affine_acceleration) const override;

	friend class SampledRenderShape;
	friend class SampledCollisionShape;
	friend class SampledObjectGravity;

protected:
	int _num_points;
	VectorXd _mass;
	MatrixXi _edge_topo;
	MatrixXi _face_topo;
	MatrixXi _tet_topo;
};

class ReducedRenderShape;

class ReducedObject : public ConcreteObject {
public:
	ReducedObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, Object* proxy, const SparseMatrixXd& base, const VectorXd& shift)
		: ConcreteObject(render_shape, collision_shape, x), _proxy(proxy), _base(base), _shift(shift) {}

	void Initialize() override {
		Object::Initialize();
		_proxy->Initialize();
	}

	void SetCoordinate(const Ref<const VectorXd> &x) override {
		ConcreteObject::SetCoordinate(x);
		_proxy->SetCoordinate(_base * x + _shift);
	}
	void SetVelocity(const Ref<const VectorXd> &v) override {
		ConcreteObject::SetVelocity(v);
		_proxy->SetVelocity(_base * v);
	}

	double GetMaxVelocity(const Ref<const VectorXd> &v) const override {return _proxy->GetMaxVelocity(_base * v);}

	Vector3d GetUnnormalizedMassCenter() const override {
		return _proxy->GetUnnormalizedMassCenter();
	}

	Matrix3d GetInertialTensor() const override {
		return _proxy->GetInertialTensor();
	}

	void GetMass(COO &coo, int x_offset, int y_offset) const override;
	double GetTotalMass() const override {return _proxy->GetTotalMass();}

	double GetPotential(const Ref<const VectorXd> &x) const override {return _proxy->GetPotential(_base * x + _shift);}
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override { return _base.transpose() * _proxy->GetPotentialGradient(_base * x + _shift);}
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override;

	void AddExternalForce(ExternalForce*force) override {_proxy->AddExternalForce(force);}
	VectorXd GetExternalForce() const override {return _base.transpose() * _proxy->GetExternalForce();}
	Vector3d GetTotalExternalForce() const override {return _proxy->GetTotalExternalForce();}

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override {return _base.transpose() * _proxy->GetInertialForce(v, a, omega, alpha, rotation);}
	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Matrix3d &affine, const Matrix3d &affine_velocity, const Matrix3d &affine_acceleration) const override {
		return _base.transpose() * _proxy->GetInertialForce(v, a, affine, affine_velocity, affine_acceleration);
	}

	friend class ReducedRenderShape;

protected:
	Object* _proxy;
	const SparseMatrixXd _base;
	const VectorXd _shift;
};

class FixedObject : public ConcreteObject {
public:
	FixedObject(const json& config);
	FixedObject(RenderShape* render_shape, CollisionShape* collision_shape)
		: ConcreteObject(render_shape, collision_shape, VectorXd(0)) {}
	
	double GetMaxVelocity(const Ref<const VectorXd> &v) const override {return 0;}
	void GetMass(COO &coo, int x_offset, int y_offset) const override {}
	double GetTotalMass() const override {return 0;}

	double GetPotential(const Ref<const VectorXd> &x) const override {return 0;}
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override {return VectorXd(0);}
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override {}

	Vector3d GetTotalExternalForce() const override {return Vector3d::Zero();}

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override {return VectorXd(0);}
};

DECLARE_XXX_FACTORY(Object)

#endif //FEM_OBJECT_H
