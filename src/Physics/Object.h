//
// Created by hansljy on 10/7/22.
//

#ifndef FEM_OBJECT_H
#define FEM_OBJECT_H

#include "EigenAll.h"
#include "Pattern.h"
#include "ExternalForce/ExternalForce.h"

class Object {
public:
    Object(const VectorXd& x) : _extra_force(x.size()), _x(x), _v(x.size()) {
        _v.setZero();
        _extra_force.setZero();
    }
    Object(const VectorXd& x, const VectorXd& v) : _extra_force(x.size()), _x(x), _v(v) {
        _extra_force.setZero();
    }

    int GetDOF() const {
        return _x.size();
    }

    const VectorXd & GetCoordinate() const {
        return _x;
    }

    const VectorXd & GetVelocity() const {
        return _v;
    }

    virtual void SetCoordinate(const VectorXd& x) {
        _x = x;
    }

    virtual void SetVelocity(const VectorXd& v) {
        _v = v;
    }

    // General mass (sparse matrix form)
    virtual void GetMass(COO &coo, int x_offset, int y_offset) const = 0;           // General mass (coo form)
    virtual double GetTotalMass() const = 0;

    virtual double GetPotential() const {
        return GetPotential(_x);
    }
    virtual double GetPotential(const Ref<const VectorXd>& x) const = 0;
    virtual VectorXd GetPotentialGradient() const {
        return GetPotentialGradient(_x);
    }
    virtual VectorXd GetPotentialGradient(const Ref<const VectorXd>& x) const = 0;
    virtual void GetPotentialHessian(COO &coo, int x_offset, int y_offset) const {
        GetPotentialHessian(_x, coo, x_offset, y_offset);
    }
    virtual void GetPotentialHessian(const Ref<const VectorXd>& x, COO& coo, int x_offset, int y_offset) const = 0;

    virtual VectorXd
    GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha,
                     const Matrix3d &rotation) const = 0;

    virtual void AddExternalForce(const ExternalForce& force);

    VectorXd GetExternalForce(const Matrix3d& rotation, const Vector3d& position) const;
    virtual Vector3d GetTotalExternalForce(const Matrix3d &rotation, const Vector3d &position) const = 0;

    virtual void GetRenderShape(MatrixXd& vertices, MatrixXi& topo) const = 0;
    virtual void GetCollisionShape(const Ref<const VectorXd> &x, MatrixXd &vertices, MatrixXi &face_topo, MatrixXi &edge_topo) const {}

    virtual int GetConstraintSize() const;
    virtual VectorXd GetInnerConstraint(const VectorXd &x) const;
    virtual void GetInnerConstraintGradient(const VectorXd &x, COO &coo, int x_offset, int y_offset) const;

    virtual ~Object();
    Object(const Object& rhs);
    Object& operator=(const Object& rhs) = delete;

    BASE_DECLARE_CLONE(Object)

public:
    Vector3d _frame_x = Vector3d::Zero();
    Matrix3d _frame_rotation = Matrix3d::Identity();
    VectorXd _extra_force;

protected:
    VectorXd _x;
    VectorXd _v;

    std::vector<const ExternalForce*> _external_forces;
};

class Shape;

class ShapedObject : virtual public Object {
public:
    explicit ShapedObject(const Shape &shape);
    void GetRenderShape(Eigen::MatrixXd &vertices, Eigen::MatrixXi &topo) const override;

    ~ShapedObject() override;
    ShapedObject(const ShapedObject& rhs);
    ShapedObject& operator=(const ShapedObject& rhs) = delete;

protected:
    const Shape* _shape;
};

class SampledObjectGravity;
class TreeTrunkGravity;

class SampledObject : virtual public Object {
public:
    explicit SampledObject(const VectorXd& mass);
    void GetMass(COO &coo, int x_offset, int y_offset) const override;
    double GetTotalMass() const override;
    Vector3d GetTotalExternalForce(const Matrix3d &rotation, const Vector3d &position) const override;

    VectorXd GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha,
                              const Matrix3d &rotation) const override;

    ~SampledObject() override = default;
    MIDDLE_DECLARE_CLONE(Object)

    friend class SampledObjectGravity;
    friend class TreeTrunkGravity;

protected:
    const VectorXd _mass;
};

DECLARE_XXX_FACTORY(Object)

#endif //FEM_OBJECT_H
