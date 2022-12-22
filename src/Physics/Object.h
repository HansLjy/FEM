//
// Created by hansljy on 10/7/22.
//

#ifndef FEM_OBJECT_H
#define FEM_OBJECT_H

#include "EigenAll.h"
#include "Pattern.h"
#include "ExternalForce/ExternalForce.h"

class Physics;
class RenderShape;
class CollisionShape;

class DecomposedObject;

class Object {
public:
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

    virtual void AddExternalForce(const ExternalForce& force);
    virtual VectorXd GetExternalForce() const;
    virtual Vector3d GetTotalExternalForce() const;

	virtual VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const = 0;

    /* Render relevant */
    virtual void GetRenderShape(MatrixXd& vertices, MatrixXi& topo) const;

    /* Collision relevant */
	void ComputeCollisionShape() {ComputeCollisionShape(_x);}
    virtual void ComputeCollisionShape(const Ref<const VectorXd>& x);
    virtual const MatrixXd& GetCollisionVertices() const;
    virtual const MatrixXi& GetCollisionEdgeTopo() const;
    virtual const MatrixXi& GetCollisionFaceTopo() const;

	/* Frame relevant */
	virtual void SetFrame(const Matrix3d& rotation, const Vector3d& shift);
	virtual Vector3d GetFrameX() const;
	virtual Matrix3d GetFrameRotation() const;

    virtual ~Object();
    Object(const Object& rhs);
    Object& operator=(const Object& rhs) = delete;

    BASE_DECLARE_CLONE(Object)
	friend class DecomposedObject;

protected:
    RenderShape* _render_shape = nullptr;
    CollisionShape* _collision_shape = nullptr;
	std::vector<ExternalForce*> _external_forces;

    VectorXd _x;
    VectorXd _v;
	int _dof;
};

class SampledObject : public Object {
public:
	SampledObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, const VectorXd& mass);
	SampledObject(RenderShape* render_shape, CollisionShape* collision_shape, const VectorXd& x, const VectorXd& v, const VectorXd& mass);
	
	double GetMaxVelocity(const Ref<const VectorXd> &v) const override;

	void GetMass(COO &coo, int x_offset, int y_offset) const override;
	double GetTotalMass() const override;

	Vector3d GetTotalExternalForce() const override;

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override;

protected:
	VectorXd _mass;
};

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

	void AddExternalForce(const ExternalForce &force) override {_proxy->AddExternalForce(force);}
	VectorXd GetExternalForce() const override {return _base.transpose() * _proxy->GetExternalForce();}
	Vector3d GetTotalExternalForce() const override {return _proxy->GetTotalExternalForce();}

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override {return _base.transpose() * _proxy->GetInertialForce(v, a, omega, alpha, rotation);}

	void SetFrame(const Matrix3d &rotation, const Vector3d &shift) override {
		Object::SetFrame(rotation, shift);
		_proxy->SetFrame(rotation, shift);
	}

protected:
	Object* _proxy;
	const SparseMatrixXd _base;
	const VectorXd _shift;
};

class DecomposedObject : public Object {
public:
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

	void AddExternalForce(const ExternalForce &force) override {_proxy->AddExternalForce(force);}
	VectorXd GetExternalForce() const override;
	Vector3d GetTotalExternalForce() const override {return _proxy->GetTotalExternalForce();}

	VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha, const Matrix3d &rotation) const override {return _proxy->GetInertialForce(v, a, omega, alpha, rotation);}

	void GetRenderShape(MatrixXd &vertices, MatrixXi &topo) const override {_proxy->GetRenderShape(vertices, topo);}

	void ComputeCollisionShape(const Ref<const VectorXd> &x) override;
	const MatrixXd & GetCollisionVertices() const override {return _proxy->GetCollisionVertices();}
	const MatrixXi & GetCollisionEdgeTopo() const override {return _proxy->GetCollisionEdgeTopo();}
	const MatrixXi & GetCollisionFaceTopo() const override {return _proxy->GetCollisionFaceTopo();}

	/* Frame relevant */
	void SetFrame(const Matrix3d &rotation, const Vector3d &shift) override;
	Vector3d GetFrameX() const override;
	Matrix3d GetFrameRotation() const override;

	/* Domain relevant */
	/**
	 * @brief Calculate frame of the children
	 * @note This is a non-recursive function
	 */
	virtual void CalculateChildrenFrame() = 0;
	/**
	 * @brief Aggregate infomation from bottom to top
	 * @note This is a recursive function
	 */
	void Aggregate();
	virtual void AddChild(const DecomposedObject& child, const json& position) = 0;
	
	~DecomposedObject() override;
	DecomposedObject(const DecomposedObject& rhs);

protected:
	Object* _proxy;

	std::vector<DecomposedObject*> _children;
	std::vector<SparseMatrixXd> _children_projections;
	
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

DECLARE_XXX_FACTORY(Object)

#endif //FEM_OBJECT_H
