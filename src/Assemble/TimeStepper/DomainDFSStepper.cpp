//
// Created by hansljy on 11/28/22.
//

#include "DomainDFSStepper.h"

DomainDFSStepper::DomainDFSStepper(const nlohmann::json &config) {
    const auto& integrator_config = config["integrator"];
    _integrator = IntegratorFactory::GetIntegrator(integrator_config["type"], integrator_config);
}

void DomainDFSStepper::Step(double h) const {
    auto& root_domain = dynamic_cast<Domain&>(*_system);
    root_domain.BottomUpCalculation();
    StepNonRoot(root_domain, h);
}

void DomainDFSStepper::StepNonRoot(Domain &domain, double h) const {
    domain.TopDownCalculationPrev();
    const auto& domain_target = domain.GetTarget();
    VectorXd v = domain_target->GetVelocity();
    _integrator->Step(*domain_target, h);
    if (!domain._subdomains.empty()) {
        domain.CalculateSubdomainFrame((domain_target->GetVelocity() - v) / h);
    }
    for (auto& subdomain : domain._subdomains) {
        StepNonRoot(*subdomain, h);
    }
}

DomainDFSStepper::~DomainDFSStepper() {
    delete _integrator;
}