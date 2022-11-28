//
// Created by hansljy on 11/9/22.
//

#include "DomainIntegrator.h"

void DomainIntegrator::Step(Target &target, double h) const {
    auto& root_domain_target = dynamic_cast<DomainTarget&>(target);
    root_domain_target.BottomUpCalculation();
    StepNonRoot(root_domain_target, h);
}

void DomainIntegrator::StepNonRoot(DomainTarget &domain_target, double h) const {
    domain_target.TopDownCalculationPrev();
    VectorXd v = domain_target.GetVelocity();
    _integrator->Step(domain_target, h);
    domain_target.TopDownCalculationAfter((domain_target.GetVelocity() - v) / h);
    for (auto& subdomain : domain_target.GetSubdomains()) {
        StepNonRoot(*dynamic_cast<DomainTarget*>(subdomain->GetTarget()), h);
    }
}

DomainIntegrator::DomainIntegrator(const json &config) {
    const auto& internal_integrator_config = config["internal-integrator"];
    _integrator = IntegratorFactory::GetIntegrator(internal_integrator_config["type"], internal_integrator_config);
}

DomainIntegrator::~DomainIntegrator() noexcept {
    delete _integrator;
}