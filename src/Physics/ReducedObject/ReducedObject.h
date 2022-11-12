//
// Created by hansljy on 10/18/22.
//

#ifndef FEM_REDUCEDOBJECT_H
#define FEM_REDUCEDOBJECT_H

#include "Object.h"
#include "EigenAll.h"

class ReducedObject : public Object {
public:
    ReducedObject(const VectorXd &x, const Object &proxy, const SparseMatrixXd &base, const VectorXd &shift);

    void SetCoordinate(const Eigen::VectorXd &x) override;
    void SetVelocity(const Eigen::VectorXd &v) override;
    void SetProxyCoordinate();
    void SetProxyVelocity();

    // note: if any derived class found that mass is a constant matrix
    //       it should override it. This function will evaluate every time
    void GetMass(COO &coo, int x_offset, int y_offset) const override;
    double GetTotalMass() const override;

    double GetPotential() const override;
    double GetPotential(const Ref<const Eigen::VectorXd> &x) const override;
    VectorXd GetPotentialGradient() const override;
    void GetPotentialHessian(COO &coo, int x_offset, int y_offset) const override;

    void AddExternalForce(const ExternalForce &force) override;

    double GetExternalEnergy(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &position) const override;
    double GetExternalEnergy(const Ref<const Eigen::VectorXd> &x, const Eigen::Matrix3d &rotation, const Eigen::Vector3d &position) const override;
    VectorXd GetExternalEnergyGradient(const Matrix3d &rotation, const Vector3d &position) const override;
    Vector3d GetTotalExternalForce(const Matrix3d &rotation, const Vector3d &position) const override;
    void GetExternalEnergyHessian(const Matrix3d &rotation, const Vector3d &position, COO &coo, int x_offset,
                                  int y_offset) const override;

    VectorXd GetInertialForce(const Eigen::Vector3d &v, const Eigen::Vector3d &a, const Eigen::Vector3d &omega,
                              const Eigen::Vector3d &alpha, const Eigen::Matrix3d &rotation) const override;

    int GetConstraintSize() const override;
    VectorXd GetInnerConstraint(const VectorXd &x) const override;
    void GetInnerConstraintGradient(const VectorXd &x, COO &coo, int x_offset, int y_offset) const override;

    void GetShape(Eigen::MatrixXd &vertices, Eigen::MatrixXi &topo) const override;

    MIDDLE_DECLARE_CLONE(Object)

    ~ReducedObject() override;
    ReducedObject(const ReducedObject& rhs);
    ReducedObject& operator=(const ReducedObject& rhs);

protected:
    Object* _proxy;
    SparseMatrixXd _base;
    VectorXd _shift;
};

#endif //FEM_REDUCEDOBJECT_H
