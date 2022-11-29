//
// Created by hansljy on 11/29/22.
//

#include "DomainBFSStepper.h"

DomainBFSStepper::DomainBFSStepper(const nlohmann::json &config) {
    const auto& integrator_config = config["integrator"];
    _integrator = IntegratorFactory::GetIntegrator(integrator_config["type"], integrator_config);
}

void DomainBFSStepper::Step(double h) const {
    // TODO
}

DomainBFSStepper::~DomainBFSStepper() noexcept {
    delete _integrator;
}
