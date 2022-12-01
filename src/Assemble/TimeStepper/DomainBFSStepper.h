//
// Created by hansljy on 11/29/22.
//

#ifndef FEM_DOMAINBFSSTEPPER_H
#define FEM_DOMAINBFSSTEPPER_H

#include "TimeStepper.h"
#include "Integrator/Integrator.h"
#include "Domain.h"

class DomainBFSStepper : public TimeStepper {
public:
    explicit DomainBFSStepper(const json& config);
    void Bind(System &system) override;
    void Step(double h) const override;

    ~DomainBFSStepper() noexcept override;

protected:
    Integrator* _integrator;
    std::vector<Domain*> _domains;
    std::vector<int> _level_bar;
    int _level;
};

class GroupDomainTarget : public Target {
public:
    GroupDomainTarget(const std::vector<Domain*>& domains, int begin, int end);

    int GetDOF() const override;

    void GetCoordinate(Ref<Eigen::VectorXd> x) const override;
    void GetVelocity(Ref<Eigen::VectorXd> v) const override;

    void SetCoordinate(const Ref<const Eigen::VectorXd> &x) override;
    void SetVelocity(const Ref<const Eigen::VectorXd> &v) override;

    void GetMass(COO &coo, int offset_x, int offset_y) const override;

    double GetPotentialEnergy() const override;
    double GetPotentialEnergy(const Ref<const Eigen::VectorXd> &x) const override;
    void GetPotentialEnergyGradient(Ref<Eigen::VectorXd> gradient) const override;
    void GetPotentialEnergyGradient(const Ref<const Eigen::VectorXd> &x, Ref<Eigen::VectorXd> gradient) const override;
    void GetPotentialEnergyHessian(COO &coo, int offset_x, int offset_y) const override;
    void GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const override;

    void GetExternalForce(Ref<Eigen::VectorXd> force) const override;

    VectorXd GetConstraint(const Eigen::VectorXd &x) const override;
    void GetConstraintGradient(SparseMatrixXd &gradient, const Eigen::VectorXd &x) const override;

    ~GroupDomainTarget() = default;

protected:
    const std::vector<Domain*>& _domains;
    int _begin, _end;
    int _dof;
};

#endif //FEM_DOMAINBFSSTEPPER_H
