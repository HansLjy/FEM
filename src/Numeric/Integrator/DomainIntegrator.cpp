//
// Created by hansljy on 11/9/22.
//

#include "DomainIntegrator.h"

void DomainIntegrator::Step(Target &target, double h) const {
    auto& root_domain = dynamic_cast<Domain&>(target);
    root_domain.CalculateTotalMass();
    root_domain.CalculateTotalExternalForce();
    StepNonRoot(root_domain, h);
}

void DomainIntegrator::StepNonRoot(Domain &domain, double h) const {
    domain.Preparation();
    VectorXd v = domain.GetVelocity();
    _integrator->Step(domain, h);
    if (!domain._subdomains.empty()) {
        domain.CalculateSubdomainFrame((domain.GetVelocity() - v) / h);
    }
    for (auto& subdomain : domain._subdomains) {
        StepNonRoot(*subdomain, h);
    }
}

DomainIntegrator::DomainIntegrator(const json &config) {
    const auto& internal_integrator_config = config["internal-integrator"];
    _integrator = IntegratorFactory::GetIntegrator(internal_integrator_config["type"], internal_integrator_config);
}

DomainIntegrator::~DomainIntegrator() noexcept {
    delete _integrator;
}