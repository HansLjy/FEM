//
// Created by hansljy on 11/28/22.
//

#include "SystemStepper.h"

SystemStepper::SystemStepper(const json &config) {
    const auto& integrator_config = config["integrator"];
    _integrator = IntegratorFactory::GetIntegrator(integrator_config["type"], integrator_config);
}

void SystemStepper::Step(double h) const {
    _integrator->Step(*_system->GetTarget(), h);
}

SystemStepper::~SystemStepper() {
    delete _integrator;
}

