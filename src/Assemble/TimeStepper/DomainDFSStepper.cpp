//
// Created by hansljy on 11/28/22.
//

#include "DomainDFSStepper.h"

DomainDFSStepper::DomainDFSStepper(const nlohmann::json &config) : TimeStepper(config) {
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
    DomainTarget domain_target(domain);
    VectorXd v(domain_target.GetDOF()), v_new(domain_target.GetDOF());
    domain_target.GetVelocity(v);
    _integrator->Step(domain_target, h);
    if (!domain._subdomains.empty()) {
        domain_target.GetVelocity(v_new);
        domain.CalculateSubdomainFrame((v_new - v) / h);
    }
    for (auto& subdomain : domain._subdomains) {
        StepNonRoot(*subdomain, h);
    }
}

DomainDFSStepper::~DomainDFSStepper() {
    delete _integrator;
}