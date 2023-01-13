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

class DecomposedObject;


enum class CollisionAssemblerType {
	kNull,
	kIndex,
};

class Object {
public:
	Object() = default;
    Object(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x);
    Object(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, const VectorXd& v);

	virtual int GetDOF() const;
    virtual const VectorXd & GetCoordinate() const;
    virtual const VectorXd & GetVelocity() const;
    virtual void SetCoordinate(const Ref<const VectorXd>& x);
    virtual void SetVelocity(const Ref<const VectorXd>& v);

	virtual double GetMaxVelocity(const Ref<const VectorXd>& v) const = 0;

    /* Physics relevant */
    virtual void GetMass(COO& coo, int x_offset, int y_offset) const = 0;
    virtual double GetTotalMass() const = 0;

    virtual double GetPotential(const Ref<const VectorXd>& x) const = 0;
    virtual VectorXd GetPotentialGradient(const Ref<const VectorXd>& x) const = 0;
    virtual void GetPotentialHessian(const Ref<const VectorXd>& x, COO& coo, int x_offset, int y_offset) const = 0;

    virtual void AddExternalForce(ExternalForce* force);
    virtual VectorXd GetExternalForce() const;
    virtual Vector3d GetTotalExternalForce() const = 0;

	virtual VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const = 0;

    /* Render relevant */
    virtual void GetRenderShape(MatrixXd& vertices, MatrixXi& topo) const;

    /* Collision relevant */
	void ComputeCollisionShape() {ComputeCollisionShape(_x);}
    virtual void ComputeCollisionShape(const Ref<const VectorXd>& x);
	virtual CollisionAssemblerType GetCollisionAssemblerType() const;
	virtual Vector3d GetCollisionVertexVelocity(const Ref<const VectorXd>& v, int idx);
    virtual const MatrixXd& GetCollisionVertices() const;
    virtual const MatrixXi& GetCollisionEdgeTopo() const;
    virtual const MatrixXi& GetCollisionFaceTopo() const;
	virtual const SparseMatrixXd& GetVertexProjectionMatrix() const;

	/* Frame relevant */
	virtual Vector3d GetFrameX() const;
	virtual Matrix3d GetFrameRotation() const;

	virtual bool IsDcomposed() { return false;}
	virtual void Initialize();

    virtual ~Object();
    Object(const Object& rhs) = delete;
    Object& operator=(const Object& rhs) = delete;

	friend class DecomposedObject;

protected:
    RenderShape* _render_shape = nullptr;
    CollisionShape* _collision_shape = nullptr;
	std::vector<ExternalForce*> _external_forces;

    VectorXd _x;
    VectorXd _v;
	int _dof;
};

class SampledObjectGravity;

class SampledObject : public Object {
public:
	SampledObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, const VectorXd& mass);
	SampledObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, const VectorXd& v, const VectorXd& mass);
	
	double GetMaxVelocity(const Ref<const VectorXd> &v) const override;

	void GetMass(COO &coo, int x_offset, int y_offset) const override;
	double GetTotalMass() const override;

	Vector3d GetTotalExternalForce() const override;

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override;

	friend class SampledObjectGravity;

protected:
	VectorXd _mass;
};

class ReducedRenderShape;

class ReducedObject : public Object {
public:
	ReducedObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, Object* proxy, const SparseMatrixXd& base, const VectorXd& shift)
		: Object(render_shape, collision_shape, x), _proxy(proxy), _base(base), _shift(shift) {}

	void SetCoordinate(const Ref<const VectorXd> &x) override {
		Object::SetCoordinate(x);
		_proxy->SetCoordinate(_base * x + _shift);
	}
	void SetVelocity(const Ref<const VectorXd> &v) override {
		Object::SetVelocity(v);
		_proxy->SetVelocity(_base * v);
	}

	double GetMaxVelocity(const Ref<const VectorXd> &v) const override {return _proxy->GetMaxVelocity(_base * v);}

	void GetMass(COO &coo, int x_offset, int y_offset) const override;
	double GetTotalMass() const override {return _proxy->GetTotalMass();}

	double GetPotential(const Ref<const VectorXd> &x) const override {return _proxy->GetPotential(_base * x + _shift);}
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override { return _base.transpose() * _proxy->GetPotentialGradient(_base * x + _shift);}
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override;

	void AddExternalForce(ExternalForce*force) override {_proxy->AddExternalForce(force);}
	VectorXd GetExternalForce() const override {return _base.transpose() * _proxy->GetExternalForce();}
	Vector3d GetTotalExternalForce() const override {return _proxy->GetTotalExternalForce();}

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override {return _base.transpose() * _proxy->GetInertialForce(v, a, omega, alpha, rotation);}

	friend class ReducedRenderShape;

protected:
	Object* _proxy;
	const SparseMatrixXd _base;
	const VectorXd _shift;
};

class System;
class DecomposedTreeTrunk;

class DecomposedObject : public Object {
public:
	explicit DecomposedObject(Object* proxy, const json& config);

	int GetDOF() const override {return _proxy->GetDOF();};
	const VectorXd & GetCoordinate() const override {return _proxy->GetCoordinate();}
	const VectorXd & GetVelocity() const override {return _proxy->GetVelocity();}
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

	void GetRenderShape(MatrixXd &vertices, MatrixXi &topo) const override {_proxy->GetRenderShape(vertices, topo);}

	void ComputeCollisionShape(const Ref<const VectorXd> &x) override;
	Vector3d GetCollisionVertexVelocity(const Ref<const VectorXd>& v, int idx) override {return _proxy->GetCollisionVertexVelocity(v, idx);}
	const MatrixXd & GetCollisionVertices() const override {return _proxy->GetCollisionVertices();}
	const MatrixXi & GetCollisionEdgeTopo() const override {return _proxy->GetCollisionEdgeTopo();}
	const MatrixXi & GetCollisionFaceTopo() const override {return _proxy->GetCollisionFaceTopo();}
	const SparseMatrixXd & GetVertexProjectionMatrix() const override {return _proxy->GetVertexProjectionMatrix();}

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
	void AddChild(DecomposedObject& child, const json& position);

	bool IsDcomposed() override {return true;}
	void Initialize() override;
	
	~DecomposedObject() override;
	DecomposedObject(const DecomposedObject& rhs);

	friend class System;
	friend class DecomposedTreeTrunk;

protected:
	Object* _proxy;

	std::vector<DecomposedObject*> _children;
	std::vector<Matrix3d> _children_rest_rotations;
	std::vector<SparseMatrixXd> _children_projections; // this will be set by derived class during initialization
	
	std::vector<MatrixXd> _extra_vertices;
	std::vector<MatrixXi> _extra_edge_topos;
	std::vector<MatrixXi> _extra_face_topos;

	MatrixXd _collision_vertices;
	MatrixXi _collision_edge_topo;
	MatrixXi _collision_face_topo;

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

class FixedObject : public Object {
public:
	FixedObject(const json& config);
	FixedObject(RenderShape* render_shape, CollisionShape* collision_shape)
		: Object(render_shape, collision_shape, VectorXd(0)) {}
	
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
DECLARE_XXX_FACTORY(DecomposedObject)

#endif //FEM_OBJECT_H
