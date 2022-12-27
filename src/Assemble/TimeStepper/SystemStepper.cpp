//
// Created by hansljy on 11/28/22.
//

#include "SystemStepper.h"
#include "Target.h"

SystemStepper::SystemStepper(const json &config) : TimeStepper(config) {
    const auto& integrator_config = config["integrator"];
    _integrator = IntegratorFactory::GetIntegrator(integrator_config["type"], integrator_config);
}

void SystemStepper::Bind(System &system) {
    TimeStepper::Bind(system);
    _target = new Target(system._objs);
}

void SystemStepper::Step(double h) const {
    _integrator->Step(*_target, h);
}

SystemStepper::~SystemStepper() {
    delete _integrator;
}