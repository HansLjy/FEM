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
    Object(const VectorXd& x) : _x(x), _v(x.size()) {
        _v.setConstant(0);
    }
    Object(const VectorXd& x, const VectorXd& v) : _x(x), _v(v) {}

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

    double GetEnergy(const Matrix3d &rotation, const Vector3d &position) const;
    VectorXd GetEnergyGradient(const Matrix3d &rotation, const Vector3d &position) const;
    void
    GetEnergyHessian(const Matrix3d &rotation, const Vector3d &position, COO &coo, int x_offset, int y_offset) const;

    virtual double GetPotential() const = 0;
    virtual VectorXd GetPotentialGradient() const = 0;
    virtual void GetPotentialHessian(COO &coo, int x_offset, int y_offset) const = 0;

    virtual VectorXd
    GetInertialForce(const Vector3d &v, const Vector3d &a, const Vector3d &omega, const Vector3d &alpha,
                     const Matrix3d &rotation) const = 0;

    virtual void AddExternalForce(const ExternalForce& force);

    virtual double GetExternalEnergy(const Matrix3d &rotation, const Vector3d &position) const;
    virtual VectorXd GetExternalEnergyGradient(const Matrix3d &rotation, const Vector3d &position) const;
    virtual Vector3d GetTotalExternalForce(const Matrix3d &rotation, const Vector3d &position) const = 0;
    virtual void GetExternalEnergyHessian(const Matrix3d &rotation, const Vector3d &position, COO &coo, int x_offset,
                                          int y_offset) const;

    virtual void GetShape(MatrixXd& vertices, MatrixXi& topo) const = 0;

    virtual int GetConstraintSize() const;
    virtual VectorXd GetInnerConstraint(const VectorXd &x) const;
    virtual void GetInnerConstraintGradient(const VectorXd &x, COO &coo, int x_offset, int y_offset) const;

    virtual ~Object();
    Object(const Object& rhs);
    Object& operator=(const Object& rhs) = delete;

    BASE_DECLARE_CLONE(Object)

protected:
    VectorXd _x;
    VectorXd _v;

    std::vector<const ExternalForce*> _external_forces;
};

class Shape;

class ShapedObject : virtual public Object {
public:
    explicit ShapedObject(const Shape &shape);
    void GetShape(Eigen::MatrixXd &vertices, Eigen::MatrixXi &topo) const override;

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
