//
// Created by hansljy on 11/9/22.
//

#ifndef FEM_DOMAIN_H
#define FEM_DOMAIN_H

#include "EigenAll.h"
#include "Object.h"
#include "PhysicsSystem.h"
#include "InertialSystem.h"
#include "nlohmann/json.hpp"

using nlohmann::json;

class DomainIntegrator;
class DomainIterator;

class Domain : public PhysicsSystem {
public:
    Domain(const json& config);
    void Initialize(const json& config);

    /* Target Part */
    VectorXd GetCoordinate() const override {return _system.GetCoordinate();}
    VectorXd GetVelocity() const override {return _system.GetVelocity();}

    void SetCoordinate(const VectorXd& x) override {_system.SetCoordinate(x);}
    void SetVelocity(const VectorXd& v) override {_system.SetVelocity(v);}

    void GetMass(SparseMatrixXd& mass) const override;

    double GetEnergy() const override;
    VectorXd GetEnergyGradient() const override;
    void GetEnergyHessian(SparseMatrixXd& hessian) const override {_system.GetEnergyHessian(_frame_rotation, _frame_x, hessian);}

    VectorXd GetConstraint(const VectorXd &x) const override {return _system.GetConstraint(x);}
    void GetConstraintGradient(SparseMatrixXd &gradient, const VectorXd &x) const override {_system.GetConstraintGradient(gradient, x);}

    /* Object Collection Part */
    int AddObject(const Object& obj, const std::string& name) override {return _system.AddObject(obj, name);}
    const Object* GetObject(int idx) const override {return _system.GetObject(idx);}
    Object * GetObject(int idx) override {return _system.GetObject(idx);}
    void UpdateSettings(const json &config) override;

    int AddConstraint(const Constraint& constraint) override {return _system.AddConstraint(constraint);}
    int GetIndex(const std::string& name) const override {return _system.GetIndex(name);}

    int GetOffset(int idx) const override {return _system.GetOffset(idx);};

    ObjectIterator* GetIterator() override;

    /* domain */

    void SetFrame(const Vector3d& x, const Vector3d& v, const Vector3d& a,
                  const Vector3d& angular_velocity, const Vector3d& angular_acceleration,
                  const Matrix3d& rotation);

    /**
     * Calculate x, v, a, omega, alpha, R of subdomain
     * and store these infos into subdomain
     *
     * @param a The acceleration of objects in this domain.
     *          We need this because we don't maintain
     *          acceleration of single objects.
     */
    virtual void CalculateSubdomainFrame(const VectorXd& a) = 0;

    /**
     * Calculate the total mass of current domain, this will
     * update the total mass of all children in this subtree
     */
    void CalculateTotalMass();

    /**
     * Calculate the total external force of current domain
     * this will update all children in this subtree
     */
    void CalculateTotalExternalForce();


    void Preparation();
    void CalculateInterfaceForce();
    void CalculateInertialForce();
    void CalculateLumpedMass();

    virtual SparseMatrixXd GetSubdomainProjection(const json& position) = 0;
    virtual void RecordSubdomain(const json& position) = 0;
    void AddSubdomain(const Domain& subdomain, const json& position);

    virtual ~Domain();
    Domain(const Domain& rhs);

    friend class DomainIntegrator;
    friend class DomainIterator;

    BASE_DECLARE_CLONE(Domain)

protected:
    InertialSystem _system;

    std::vector<Domain*> _subdomains;
    std::vector<SparseMatrixXd> _subdomain_projections;
    std::vector<Matrix3d> _subdomain_rest_rotations;

    Vector3d _frame_x;
    Vector3d _frame_v;
    Vector3d _frame_a;
    Vector3d _frame_angular_velocity;
    Vector3d _frame_angular_acceleration;
    Matrix3d _frame_rotation;

    double _total_mass;
    Vector3d _total_external_force;

    VectorXd _interface_force;
    VectorXd _inertial_force;
    SparseMatrixXd _lumped_mass;
};

#include <stack>

class DomainIterator : public ObjectIterator {
public:
    DomainIterator(Domain& domain);
    void Forward() override;
    Object * GetObject() override;
    Matrix3d GetRotation() override;
    Vector3d GetTranslation() override;

private:
    std::stack<Domain*> _ancestors; // the pass from root domain to current domain
    std::stack<int> _subdomain_ids; // subdomain id of the elemts in _ancestors
    Domain* _current;
    int _cur_obj_id;
    int _cur_size;
};

DECLARE_XXX_FACTORY(Domain)

#endif //FEM_DOMAIN_H
