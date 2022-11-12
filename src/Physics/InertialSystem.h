//
// Created by hansljy on 10/7/22.
//

#ifndef FEM_INERTIALSYSTEM_H
#define FEM_INERTIALSYSTEM_H

#include "PhysicsSystem.h"
#include "Object.h"
#include <string>
#include <map>
#include <vector>

class Constraint;
class SystemIterator;
class Domain;
class DomainIterator;

class InertialSystem final : public PhysicsSystem {
public:
    explicit InertialSystem(const json& config);

    /**
     * Add object into the system
     * @param obj object to be added
     * @param name name of the object (for future access)
     * @return
     *  Upon success, return the index of the object
     *  otherwise, return -1.
     *  Failure could be the result of name collision
     */
    int AddObject(const Object& obj, const std::string& name) override;


    const Object* GetObject(int idx) const override;
    Object * GetObject(int idx) override;

    void UpdateSettings(const json &config) override;

    VectorXd GetCoordinate() const override;
    VectorXd GetVelocity() const override;
    void SetCoordinate(const VectorXd& x) override;
    void SetVelocity(const VectorXd& v) override;

    void GetMass(SparseMatrixXd& mass) const override;
    double GetEnergy() const override;
    double GetEnergy(const VectorXd& x) const override;
    VectorXd GetEnergyGradient() const override;
    void GetEnergyHessian(SparseMatrixXd& hessian) const override;

    int GetOffset(int idx) const override;

    double GetTotalMass() const;
    Vector3d GetTotalExternalForce() const;

    /** Utility **/
    double GetEnergy(const Matrix3d& rotation, const Vector3d& position) const;
    double GetEnergy(const VectorXd& x, const Matrix3d& rotation, const Vector3d& position) const;
    VectorXd GetEnergyGradient(const Matrix3d& rotation, const Vector3d& position) const;
    void GetEnergyHessian(const Matrix3d &rotation, const Vector3d &position, SparseMatrixXd &hessian) const;
    Vector3d GetTotalExternalForce(const Matrix3d& rotation, const Vector3d& position) const;

    int AddConstraint(const Constraint& constraint) override;

    VectorXd GetConstraint(const VectorXd &x) const override;

    // C(x + \Delta x) = C(X) + \nabla C(x) * \Delta x
    void GetConstraintGradient(SparseMatrixXd &gradient, const VectorXd &x) const override;

    int GetIndex(const std::string& name) const override;

    ObjectIterator * GetIterator() override;

    friend class Constraint;
    friend class SystemIterator;
    friend class Domain;
    friend class DomainIterator;

protected:
    int _DOF;
    int _constraint_size;
    std::vector<Object*> _objs;
    std::vector<int> _offset;

    std::vector<Constraint*> _constraints;
    std::map<std::string, int> _index;
};

/**
 * An iterator used for objects in a system
 */
class SystemIterator : public ObjectIterator {
public:
    SystemIterator(InertialSystem& system);

    void Forward() override;
    Object * GetObject() override;
    Matrix3d GetRotation() override;
    Vector3d GetTranslation() override;

private:
    InertialSystem* _system;
    int _obj_id;
    int _cur_size;
};


#endif //FEM_INERTIALSYSTEM_H
