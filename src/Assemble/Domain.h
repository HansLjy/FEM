//
// Created by hansljy on 11/9/22.
//

#ifndef FEM_DOMAIN_H
#define FEM_DOMAIN_H

#include "EigenAll.h"
#include "Object.h"
#include "System.h"
#include "nlohmann/json.hpp"

using nlohmann::json;

class DomainTarget;
class DomainIterator;
class DomainDFSStepper;
class DomainBFSStepper;

class Domain : public System {
public:
    Domain(const json& config);

    void UpdateSettings(const json &config) override;
    std::unique_ptr<ObjectIterator> GetIterator() override;

    double GetTotalMass() const;
    Vector3d GetTotalExternalForce() const;

    /**
     * Calculate x, v, a, omega, alpha, R of subdomain
     * and store these infos into subdomain
     *
     * @param a The acceleration of objects in this domain.
     *          We need this because we don't maintain
     *          acceleration of single objects.
     */
    virtual void CalculateSubdomainFrame(const VectorXd& a) = 0;

    void BottomUpCalculation();
    void TopDownCalculationPrev();

    virtual SparseMatrixXd GetSubdomainProjection(const json& position) = 0;
    virtual void RecordSubdomain(const json& position) = 0;
    void AddSubdomain(Domain& subdomain, const json& position);

    ~Domain() override;
    Domain(const Domain& rhs) = delete;

    friend class DomainTarget;
    friend class DomainIterator;
    friend class DomainDFSStepper;
    friend class DomainBFSStepper;

protected:
    /* top down calculation */
    void CalculateInterfaceForce();
    void CalculateInertialForce();
    void CalculateLumpedMass();

    /* bottom up calculation */

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

class DomainIntegrator;

class DomainTarget : public SystemTarget {
public:
    explicit DomainTarget(Domain& domain) : SystemTarget(domain), _domain(&domain) {}
    void GetMass(SparseMatrixXd &mass) const override;
    VectorXd GetExternalForce() const override;

    friend class DomainIntegrator;

protected:
    Domain* _domain;
};

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
